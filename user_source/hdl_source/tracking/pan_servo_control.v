// First-stage horizontal visual control for the TBS-K15 position servo.
// A newly accepted measurement is consumed at most once. Lost, duplicate,
// stale or absent measurements must never keep moving the position command.
module pan_servo_control #(
    parameter integer CLK_HZ = 50000000,
    parameter integer FRAME_WIDTH = 1280,
    parameter integer CENTER_X = 640,
    parameter integer CENTER_US = 1500,
    // Conservative commissioning range; calibrate the installed mechanism.
    parameter integer MIN_US = 1400,
    parameter integer MAX_US = 1600,
    parameter integer DIRECTION = 1,
    parameter integer CONTROL_US = 40000,
    parameter integer WATCHDOG_US = 100000,
    parameter integer STOP_PIXELS = 24,
    parameter integer START_PIXELS = 40,
    parameter integer ERROR_SHIFT = 5,
    parameter integer MAX_STEP_US = 5
) (
    input wire clk,
    input wire rst_n,
    input wire enable,
    // Parent mailbox must gate sample_valid to the current enable session.
    input wire sample_valid,
    input wire sample_locked,
    input wire [10:0] sample_x,
    input wire [15:0] sample_frame_id,
    output wire pwm,
    output reg [15:0] pulse_us,
    output wire measurement_fresh,
    output reg control_active,
    output wire at_limit
);
    localparam integer DIV = CLK_HZ / 1000000;
    localparam integer DIV_BITS = (DIV < 2) ? 1 : $clog2(DIV);
    localparam integer CONTROL_BITS = (CONTROL_US < 2) ? 1 : $clog2(CONTROL_US);
    localparam integer AGE_BITS = (WATCHDOG_US < 2) ? 1 : $clog2(WATCHDOG_US + 1);
    reg [DIV_BITS-1:0] div_count;
    reg [CONTROL_BITS-1:0] control_count;
    reg [AGE_BITS-1:0] sample_age_us;
    reg have_measurement;
    reg acquired_once;
    reg have_frame;
    reg [15:0] last_frame;
    reg pending_valid;
    reg signed [11:0] pending_error;
    wire us_tick = (div_count == DIV-1);
    wire control_tick = us_tick && (control_count == CONTROL_US-1);
    wire sample_expiring = sample_age_us >= WATCHDOG_US ||
                           (us_tick && sample_age_us == WATCHDOG_US-1);
    wire [15:0] frame_delta = sample_frame_id - last_frame;
    wire ordered_sample = !have_frame || ((frame_delta != 0) && !frame_delta[15]);
    wire invalid_sample = sample_valid && (!sample_locked || sample_x >= FRAME_WIDTH);
    wire accept_sample = sample_valid && sample_locked && (sample_x < FRAME_WIDTH) && ordered_sample;
    wire signed [11:0] incoming_error = $signed({1'b0, sample_x}) - CENTER_X;
    // A measurement coincident with the control tick must be consumed now;
    // otherwise phase-aligned 25 Hz detections could starve every correction.
    wire signed [11:0] control_error=accept_sample ? incoming_error : pending_error;
    wire [11:0] error_abs = control_error[11] ? -control_error : control_error;
    wire [11:0] proportional_step = error_abs >> ERROR_SHIFT;
    wire [11:0] bounded_step = (proportional_step > MAX_STEP_US) ? MAX_STEP_US :
                              ((proportional_step == 0) ? 12'd1 : proportional_step);
    wire must_move = (error_abs > STOP_PIXELS) &&
                     (control_active || (error_abs > START_PIXELS));
    wire signed [17:0] signed_step = control_error[11] ? -$signed({6'b0, bounded_step}) :
                                                                     $signed({6'b0, bounded_step});
    wire signed [17:0] directed_step = (DIRECTION < 0) ? -signed_step : signed_step;
    wire signed [17:0] proposed_pulse = $signed({2'b0, pulse_us}) + directed_step;

    assign measurement_fresh = enable && have_measurement && (sample_age_us < WATCHDOG_US);
    assign at_limit = (pulse_us == MIN_US) || (pulse_us == MAX_US);

    // Keep PWM running across mode changes. The PWM driver applies center
    // at the next complete 20 ms pulse, never truncating the current pulse.
    wire pwm_rst_n = rst_n;
    wire unused_frame_tick;
    servo_pwm #(
        .CLK_HZ(CLK_HZ), .MIN_US(MIN_US), .MAX_US(MAX_US), .CENTER_US(CENTER_US)
    ) u_pwm (
        .clk(clk), .rst_n(pwm_rst_n), .pulse_us(pulse_us),
        .pwm(pwm), .frame_tick(unused_frame_tick)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_count <= 0;
            control_count <= 0;
            sample_age_us <= WATCHDOG_US;
            have_measurement <= 0;
            acquired_once <= 0;
            have_frame <= 0;
            last_frame <= 0;
            pending_valid <= 0;
            pending_error <= 0;
            pulse_us <= CENTER_US;
            control_active <= 0;
        end else if (!enable) begin
            div_count <= 0;
            control_count <= 0;
            sample_age_us <= WATCHDOG_US;
            have_measurement <= 0;
            acquired_once <= 0;
            have_frame <= 0;
            last_frame <= 0;
            pending_valid <= 0;
            pending_error <= 0;
            control_active <= 0;
            pulse_us <= CENTER_US; // Park both axes outside detection mode.
        end else begin
            if (us_tick) begin
                div_count <= 0;
                if (control_count == CONTROL_US-1) control_count <= 0;
                else control_count <= control_count + 1'b1;
                if (sample_age_us < WATCHDOG_US) sample_age_us <= sample_age_us + 1'b1;
            end else div_count <= div_count + 1'b1;

            if (invalid_sample) begin
                // Switching targets may invalidate the previous target at the
                // same source frame number, so invalidation bypasses ordering.
                have_measurement <= 0;
                pending_valid <= 0;
                control_active <= 0;
                sample_age_us <= WATCHDOG_US;
                if (ordered_sample) begin
                    have_frame <= 1;
                    last_frame <= sample_frame_id;
                end
            end else begin
                if (accept_sample) begin
                    have_frame <= 1;
                    last_frame <= sample_frame_id;
                    have_measurement <= 1;
                    acquired_once <= 1;
                    sample_age_us <= 0;
                    pending_valid <= 1;
                    pending_error <= incoming_error;
                end else if (sample_expiring) begin
                    have_measurement <= 0;
                    pending_valid <= 0;
                    control_active <= 0;
                end
                if (control_tick && (accept_sample || (pending_valid && !sample_expiring))) begin
                    pending_valid <= 0;
                    if (must_move) begin
                        control_active <= 1;
                        // Keep arithmetic signed until both bounds are checked.
                        if (proposed_pulse < MIN_US) pulse_us <= MIN_US;
                        else if (proposed_pulse > MAX_US) pulse_us <= MAX_US;
                        else pulse_us <= proposed_pulse[15:0];
                    end else control_active <= 0;
                end
            end
        end
    end
endmodule
