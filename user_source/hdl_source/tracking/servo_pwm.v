// Merchant PWM protocol: 20 ms period, 500..2500 us active-high pulse.
// Latch the target at the frame start, so a key change cannot truncate a pulse.
module servo_pwm #(
    parameter integer CLK_HZ = 50000000,
    parameter integer FRAME_US = 20000,
    parameter integer MIN_US = 500,
    parameter integer MAX_US = 2500,
    parameter integer CENTER_US = 1500
) (
    input wire clk,
    input wire rst_n,
    input wire [15:0] pulse_us,
    output reg pwm,
    output reg frame_tick
);
    localparam integer DIV = CLK_HZ / 1000000;
    localparam integer DIV_BITS = (DIV < 2) ? 1 : $clog2(DIV);
    localparam integer PHASE_BITS = (FRAME_US < 2) ? 1 : $clog2(FRAME_US);
    reg [DIV_BITS-1:0] div_count;
    reg [PHASE_BITS-1:0] phase_us;
    reg [15:0] active_us;
    function [15:0] clamp;
        input [15:0] value;
        begin
            if (value < MIN_US) clamp = MIN_US;
            else if (value > MAX_US) clamp = MAX_US;
            else clamp = value;
        end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_count <= 0;
            phase_us <= 0;
            active_us <= CENTER_US;
            pwm <= 0;
            frame_tick <= 0;
        end else begin
            frame_tick <= 0;
            if (div_count == DIV-1) begin
                div_count <= 0;
                if (phase_us == FRAME_US-1) begin
                    phase_us <= 0;
                    active_us <= clamp(pulse_us);
                    pwm <= 1;
                    frame_tick <= 1;
                end else begin
                    phase_us <= phase_us + 1'b1;
                    if (phase_us == active_us-1) pwm <= 0;
                end
            end else div_count <= div_count + 1'b1;
        end
    end
endmodule
