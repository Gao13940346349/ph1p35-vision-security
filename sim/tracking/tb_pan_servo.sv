`timescale 1ns/1ps
module tb_pan_servo;
    // Accelerate the control checks at an exact 1 MHz microsecond timebase.
    // A separate test checks the reused PWM with the actual 50 MHz clock.
    reg clk = 0;
    always #500 clk = ~clk;
    reg rst_n = 0, enable = 0, sample_valid = 0, sample_locked = 0;
    reg [10:0] sample_x = 640;
    reg [15:0] sample_frame_id = 0;
    wire pwm, pwm_reverse, fresh, active, limited;
    wire [15:0] pulse, pulse_reverse;
    pan_servo_control #(
        .CLK_HZ(1000000), .CONTROL_US(4000), .WATCHDOG_US(10000),
        .MIN_US(1490), .MAX_US(1510)
    ) dut (
        .clk(clk), .rst_n(rst_n), .enable(enable), .sample_valid(sample_valid),
        .sample_locked(sample_locked), .sample_x(sample_x), .sample_frame_id(sample_frame_id),
        .pwm(pwm), .pulse_us(pulse), .measurement_fresh(fresh),
        .control_active(active), .at_limit(limited)
    );
    pan_servo_control #(
        .CLK_HZ(1000000), .CONTROL_US(4000), .WATCHDOG_US(10000),
        .MIN_US(1490), .MAX_US(1510), .DIRECTION(-1)
    ) reverse_dut (
        .clk(clk), .rst_n(rst_n), .enable(enable), .sample_valid(sample_valid),
        .sample_locked(sample_locked), .sample_x(sample_x), .sample_frame_id(sample_frame_id),
        .pwm(pwm_reverse), .pulse_us(pulse_reverse), .measurement_fresh(),
        .control_active(), .at_limit()
    );
    task automatic send_sample(input [15:0] frame, input integer x, input bit locked);
        begin
            @(negedge clk);
            sample_frame_id = frame; sample_x = x; sample_locked = locked; sample_valid = 1;
            @(negedge clk); sample_valid = 0;
            #1;
        end
    endtask
    task automatic check_pulse(input integer expected, input string reason);
        begin
            if (pulse != expected || pulse_reverse != 3000-expected)
                $fatal(1, "%s: target %0d/reverse %0d, expected %0d/%0d", reason,
                       pulse, pulse_reverse, expected, 3000-expected);
        end
    endtask
    task automatic control_check(input integer expected, input string reason);
        begin
            @(negedge clk);
            while (!dut.control_tick) @(negedge clk);
            @(posedge clk); #1;
            check_pulse(expected, reason);
        end
    endtask
    initial begin
        repeat (5) @(negedge clk);
        rst_n = 1;
        send_sample(16'hfffd, 1000, 1);
        #25000000;
        if (fresh || active) $fatal(1, "Disabled mode accepted measurement or produced PWM");
        check_pulse(1500, "disabled");
        @(negedge clk); enable = 1;
        #25000000;
        if (dut.acquired_once) $fatal(1, "Enabled mode without acquired target produced PWM");

        send_sample(16'hfffe, 800, 1);
        if (!fresh || !dut.acquired_once) $fatal(1, "Locked sample did not enable fresh measurement");
        check_pulse(1500, "measurement must wait for control cadence");
        control_check(1505, "positive maximum step");
        control_check(1505, "no stale-error replay");
        control_check(1505, "watchdog freeze");
        if (fresh || active) $fatal(1, "Watchdog did not expire");

        send_sample(16'hffff, 800, 1);
        control_check(1510, "upper and reverse lower saturation");
        if (!limited) $fatal(1, "Limit not reported");
        send_sample(16'h0000, 800, 1);
        control_check(1510, "frame wrap accepted and clamp holds");
        send_sample(16'hffff, 200, 1);
        control_check(1510, "out-of-order rejected");
        send_sample(16'h0000, 200, 1);
        control_check(1510, "duplicate rejected");
        send_sample(16'h8000, 200, 1);
        control_check(1510, "ambiguous half-range rejected");

        send_sample(16'h0001, 664, 1);
        control_check(1510, "stop at inclusive 24 pixel deadzone");
        if (active) $fatal(1, "Deadzone did not stop active correction");
        send_sample(16'h0002, 680, 1);
        control_check(1510, "40 pixel restart boundary remains stopped");
        send_sample(16'h0003, 681, 1);
        control_check(1510, "41 pixel restarts but saturates");
        if (!active) $fatal(1, "Restart threshold did not engage correction");
        send_sample(16'h0004, 670, 1);
        control_check(1510, "hysteresis continues 30 pixel correction");
        if (!active) $fatal(1, "Hysteresis failed between stop and start thresholds");

        send_sample(16'h0005, 480, 1);
        send_sample(16'h0005, 480, 0);
        if (fresh || active) $fatal(1, "Same-frame invalidation retained stale target");
        control_check(1510, "switch immediately freezes pending old target");
        if (!dut.acquired_once) $fatal(1, "Target loss released PWM holding state");
        send_sample(16'h0005, 480, 1);
        control_check(1510, "same frame cannot relock after switch");
        send_sample(16'h0006, 480, 1);
        control_check(1505, "negative correction");
        send_sample(16'h0007, 480, 1);
        control_check(1500, "negative correction second measurement");
        send_sample(16'h0008, 480, 1);
        control_check(1495, "negative correction third measurement");
        send_sample(16'h0009, 480, 1);
        control_check(1490, "lower saturation");
        send_sample(16'h000a, 480, 1);
        control_check(1490, "lower clamp no unsigned wrap");
        send_sample(16'h000b, 1280, 1);
        if (fresh) $fatal(1, "Out-of-image center accepted");
        control_check(1490, "invalid center freezes");
        send_sample(16'h000b, 800, 1);
        control_check(1490, "an invalidated frame cannot be replayed as locked");

        @(negedge clk); enable = 0; #1;

        @(negedge clk); #1;
        if (fresh || active || dut.pending_valid || dut.acquired_once) $fatal(1, "Mode off did not clear measurement session");
        check_pulse(1500, "mode off returns to center");
        @(negedge clk); enable = 1;
        #25000000;
        check_pulse(1500, "new session starts centered");
        send_sample(16'h0001, 800, 1);
        control_check(1505, "new session accepts independent source sequence");
        $display("PASS pan_servo: mode gating, fresh-only cadence, watchdog, sequence wrap, switch invalidation, direction, hysteresis, signed saturation");
        $finish;
    end
    initial begin #300000000; $fatal(1, "Pan control test timed out"); end
endmodule
