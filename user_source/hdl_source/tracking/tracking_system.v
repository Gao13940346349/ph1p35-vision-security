// Connect the camera-domain tracker to the independent 50 MHz servo domain.
// Mode-off asynchronously flushes the CDC mailboxes even if CSI has stopped.
// A KEY1 request advances an epoch immediately: old in-flight measurements
// cannot move the servo while the camera is still acknowledging the switch.
module tracking_system #(
    parameter integer SECURITY_SUPPORT=0,
    parameter integer CLK_HZ=50000000,
    parameter integer IMG_WIDTH=1280,
    parameter integer IMG_HEIGHT=720,
    parameter integer KEY_DEBOUNCE_BITS=19,
    parameter integer CENTER_US=1500,
    // TBS-K15 270 deg / 2000 us: center +/-90 deg, rounded inward.
    parameter integer MIN_US=834,
    parameter integer MAX_US=2166,
    parameter integer DIRECTION=1,
    parameter integer TILT_SUPPORT=0,
    parameter integer TILT_CENTER_US=1500,
    // Center +/-20 deg, rounded inward to whole microseconds.
    parameter integer TILT_MIN_US=1352,
    parameter integer TILT_MAX_US=1648,
    parameter integer TILT_DIRECTION=1,
    parameter integer CONTROL_US=40000,
    parameter integer WATCHDOG_US=100000
)(
    input wire sys_clk,camera_clk,rst_n,enable,next_button_n,
    input wire result_strobe,result_bad,
    input wire [15:0] result_id,capture_id,
    input wire [422:0] boxes,
    // These inputs are synchronous to camera_clk; application steering off
    // also asynchronously flushes old cross-domain servo measurements.
    input wire app_mode,app_track_enable,
    input wire [46:0] app_seed,
    output wire pan_pwm,
    output wire [15:0] pulse_us,
    output wire tilt_pwm,
    output wire [15:0] tilt_pulse_us,
    output wire frame_publish,
    output wire snapshot_strobe,snapshot_bad,
    output wire [15:0] snapshot_id,
    output wire [422:0] snapshot_boxes,
    // {source frame16, state2, identity8, selected descriptor47, boxes423}
    output wire [495:0] frame_result
);
    reg [1:0] sys_reset,mode_sys_reset,mode_camera_reset;
    wire application=SECURITY_SUPPORT ? app_mode : 1'b0;
    wire application_tracking=SECURITY_SUPPORT ? app_track_enable : 1'b0;
    wire mode_arst_n=rst_n && enable;
    wire servo_arst_n=mode_arst_n && (!application || application_tracking);
    reg [1:0] servo_sys_reset;
    always @(posedge sys_clk or negedge servo_arst_n)
        if(!servo_arst_n) servo_sys_reset<=0;
        else servo_sys_reset<={servo_sys_reset[0],1'b1};
    always @(posedge sys_clk or negedge rst_n)
        if(!rst_n) sys_reset<=0; else sys_reset<={sys_reset[0],1'b1};
    always @(posedge sys_clk or negedge mode_arst_n)
        if(!mode_arst_n) mode_sys_reset<=0;
        else mode_sys_reset<={mode_sys_reset[0],1'b1};
    always @(posedge camera_clk or negedge mode_arst_n)
        if(!mode_arst_n) mode_camera_reset<=0;
        else mode_camera_reset<={mode_camera_reset[0],1'b1};

    wire key_press;
    button_toggle_ctrl #(.DEBOUNCE_BITS(KEY_DEBOUNCE_BITS)) u_next_key(
        .clk(sys_clk),.rst_n(sys_reset[1]),.button_n(next_button_n),
        .state(),.press(key_press));
    // app_mode is camera-domain: synchronize it before keypad ownership.
    reg [1:0] app_sys_sync;
    always @(posedge sys_clk or negedge rst_n)
        if(!rst_n) app_sys_sync<=0;
        else app_sys_sync<={app_sys_sync[0],application};
    wire switch_request=key_press && mode_sys_reset[1] && !app_sys_sync[1];
    reg [7:0] requested_epoch;
    // Mode-off still asserts immediately with either clock stopped. Release
    // reaches downstream registers only through their own clock's two-stage
    // synchronizer; raw mode_arst_n terminates at the synchronizers/mailboxes.
    always @(posedge sys_clk or negedge mode_sys_reset[1])
        if(!mode_sys_reset[1]) requested_epoch<=0;
        else if(switch_request) requested_epoch<=requested_epoch+1'b1;

    wire switch_received;
    wire [7:0] camera_epoch;
    tracking_mailbox #(.WIDTH(8)) u_switch_mailbox(
        .arst_n(mode_arst_n),.src_clk(sys_clk),.send(switch_request),
        .data_in(requested_epoch+8'd1),.dst_clk(camera_clk),
        .receive(switch_received),.data_out(camera_epoch));

    reg [7:0] applied_epoch;
    always @(posedge camera_clk or negedge mode_camera_reset[1])
        if(!mode_camera_reset[1]) applied_epoch<=0;
        else if(switch_received) applied_epoch<=camera_epoch;

    wire measurement_valid;
    wire [15:0] source_id,source_age;
    wire [422:0] published_boxes;
    wire [46:0] selected_box;
    wire [7:0] track_id;
    wire [1:0] track_state;
    target_tracker #(.SECURITY_SUPPORT(SECURITY_SUPPORT),.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) u_tracker(
        .clk(camera_clk),.rst_n(mode_camera_reset[1]),.enable(1'b1),
        .switch_target(switch_received),.result_strobe(result_strobe),
        .result_bad(result_bad),.result_id(result_id),.capture_id(capture_id),.boxes(boxes),
        .app_mode(application),.app_track_enable(application_tracking),.app_seed(app_seed),
        .publish(),.result_publish(frame_publish),.source_id(source_id),.source_age(source_age),
        .snapshot_strobe(snapshot_strobe),.snapshot_bad(snapshot_bad),.snapshot_id(snapshot_id),
        .publish_boxes(published_boxes),.selected_box(selected_box),
        .track_id(track_id),.session_id(),.track_state(track_state),
        .measurement_valid(measurement_valid));
    assign frame_result={source_id,track_state,track_id,selected_box,published_boxes};
    assign snapshot_boxes=published_boxes;

    wire [11:0] center_sum={1'b0,selected_box[41:31]}+{1'b0,selected_box[30:20]};
    wire [10:0] center_y_sum={1'b0,selected_box[19:10]}+{1'b0,selected_box[9:0]};
    wire [10:0] center_y={1'b0,center_y_sum[10:1]};
    reg filter_valid;
    reg [7:0] filter_id;
    reg [10:0] filtered_x;
    reg [10:0] filtered_y;
    wire [11:0] filtered_sum={1'b0,filtered_x}+{1'b0,center_sum[11:1]};
    wire [10:0] control_x=filter_valid && filter_id==track_id ? filtered_sum[11:1] : center_sum[11:1];
    wire [11:0] filtered_y_sum={1'b0,filtered_y}+{1'b0,center_y};
    wire [10:0] control_y=filter_valid && filter_id==track_id ? filtered_y_sum[11:1] : center_y;
    always @(posedge camera_clk or negedge mode_camera_reset[1]) begin
        if(!mode_camera_reset[1]) begin filter_valid<=0;filter_id<=0;filtered_x<=0;filtered_y<=0;end
        else if(switch_received || (application && !application_tracking)) filter_valid<=0;
        else if(frame_publish) begin
            filter_valid<=measurement_valid;
            filter_id<=track_id;
            if(measurement_valid) begin filtered_x<=control_x;filtered_y<=control_y;end
        end
    end
    wire receive_sample;
    // Both axes use the same target, source frame, validity and switch epoch.
    wire [46:0] servo_message;
    tracking_mailbox #(.WIDTH(47)) u_measurement_mailbox(
        .arst_n(servo_arst_n),.src_clk(camera_clk),.send(frame_publish),
        .data_in({applied_epoch,source_id,measurement_valid,control_y,control_x}),
        .dst_clk(sys_clk),.receive(receive_sample),.data_out(servo_message));
    wire current_sample=receive_sample && servo_message[46:39]==requested_epoch;
    reg [15:0] delivered_frame;
    always @(posedge sys_clk or negedge mode_sys_reset[1])
        if(!mode_sys_reset[1]) delivered_frame<=0;
        else if(current_sample && !switch_request) delivered_frame<=servo_message[38:23];
    pan_servo_control #(
        .CLK_HZ(CLK_HZ),.FRAME_WIDTH(IMG_WIDTH),.CENTER_X(IMG_WIDTH/2),
        .CENTER_US(CENTER_US),.MIN_US(MIN_US),.MAX_US(MAX_US),.DIRECTION(DIRECTION),
        .CONTROL_US(CONTROL_US),.WATCHDOG_US(WATCHDOG_US)
    ) u_pan(
        .clk(sys_clk),.rst_n(sys_reset[1]),.enable(servo_sys_reset[1]),
        .sample_valid(switch_request || current_sample),
        .sample_locked(!switch_request && servo_message[22]),
        .sample_x(servo_message[10:0]),
        .sample_frame_id(switch_request ? delivered_frame : servo_message[38:23]),
        .pwm(pan_pwm),.pulse_us(pulse_us),.measurement_fresh(),
        .control_active(),.at_limit());
    generate if(TILT_SUPPORT) begin: g_tilt
        // Reuse the scalar controller with image height as its coordinate span.
        pan_servo_control #(
            .CLK_HZ(CLK_HZ),.FRAME_WIDTH(IMG_HEIGHT),.CENTER_X(IMG_HEIGHT/2),
            .CENTER_US(TILT_CENTER_US),.MIN_US(TILT_MIN_US),.MAX_US(TILT_MAX_US),
            .DIRECTION(TILT_DIRECTION),.CONTROL_US(CONTROL_US),.WATCHDOG_US(WATCHDOG_US)
        ) u_tilt(
            .clk(sys_clk),.rst_n(sys_reset[1]),.enable(servo_sys_reset[1]),
            .sample_valid(switch_request || current_sample),
            .sample_locked(!switch_request && servo_message[22]),
            .sample_x(servo_message[21:11]),
            .sample_frame_id(switch_request ? delivered_frame : servo_message[38:23]),
            .pwm(tilt_pwm),.pulse_us(tilt_pulse_us),.measurement_fresh(),
            .control_active(),.at_limit());
    end else begin: g_no_tilt
        assign tilt_pwm=1'b0;
        assign tilt_pulse_us=TILT_CENTER_US;
    end endgenerate
endmodule
