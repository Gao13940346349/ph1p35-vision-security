`timescale 1ns/1ps
module tb_tracking_pwm;
    reg clk=0;
    always #10 clk=~clk;
    reg rst_n=0;
    reg [15:0] target=1500, tilt_target=1500;
    wire pwm, tick, tilt_pwm;
    servo_pwm #(.MIN_US(834), .MAX_US(2166)) dut (
        .clk(clk), .rst_n(rst_n), .pulse_us(target), .pwm(pwm), .frame_tick(tick)
    );
    servo_pwm #(.MIN_US(1352), .MAX_US(1648)) tilt_dut (
        .clk(clk), .rst_n(rst_n), .pulse_us(tilt_target), .pwm(tilt_pwm), .frame_tick()
    );
    time previous_rise=0;
    task automatic check_frame(input integer expected_us,input integer expected_tilt_us);
        time rise_time, fall_time, tilt_fall_time;
        begin
            @(posedge pwm); rise_time=$time;
            if (previous_rise != 0 && rise_time-previous_rise != 20000000)
                $fatal(1,"Wrong period %0t",rise_time-previous_rise);
            previous_rise=rise_time;
            #1;
            if (tilt_pwm !== 1) $fatal(1,"Tilt PWM frame start differs from pan");
            fork
                begin
                    @(negedge pwm); fall_time=$time;
                    if (fall_time-rise_time != expected_us*1000)
                        $fatal(1,"Width %0t, expected %0d us",fall_time-rise_time,expected_us);
                end
                begin
                    @(negedge tilt_pwm); tilt_fall_time=$time;
                    if (tilt_fall_time-rise_time != expected_tilt_us*1000)
                        $fatal(1,"Tilt width %0t, expected %0d us",tilt_fall_time-rise_time,expected_tilt_us);
                end
            join
        end
    endtask
    initial begin
        repeat(5) @(negedge clk);
        if (pwm !== 0 || tilt_pwm !== 0) $fatal(1,"PWM not low during reset");
        rst_n=1;
        check_frame(1500,1500);
        fork
            check_frame(1500,1500);
            begin @(posedge pwm); #200000; target=834; tilt_target=1352; end
        join
        check_frame(834,1352);
        @(negedge clk); target=2166; tilt_target=1648;
        check_frame(2166,1648);
        @(negedge clk); target=0; tilt_target=0;
        check_frame(834,1352);
        @(negedge clk); target=65535; tilt_target=65535;
        check_frame(2166,1648);
        @(posedge pwm); #200000; rst_n=0; #1;
        if (pwm !== 0 || tilt_pwm !== 0) $fatal(1,"Disable reset did not stop PWM");
        previous_rise=0;
        repeat(5) @(negedge clk); target=1500; tilt_target=1500; rst_n=1;
        check_frame(1500,1500);
        $display("PASS tracking_pwm: actual 50 MHz, exact 50 Hz, pan834/1500/2166 us and tilt1352/1500/1648 us, frame latch, clamps, reset");
        $finish;
    end
    initial begin #250000000; $fatal(1,"PWM test timed out"); end
endmodule
