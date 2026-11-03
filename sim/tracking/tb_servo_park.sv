`timescale 1ns/1ps
module tb_servo_park;
    reg clk=0; always #500 clk=~clk;
    reg rst_n=0,enable=0,valid=0;
    wire pwm; wire [15:0] pulse;
    pan_servo_control #(.CLK_HZ(1000000),.CONTROL_US(4000)) dut(
        .clk(clk),.rst_n(rst_n),.enable(enable),.sample_valid(valid),
        .sample_locked(1'b1),.sample_x(11'd900),.sample_frame_id(16'd1),
        .pwm(pwm),.pulse_us(pulse),.measurement_fresh(),.control_active(),.at_limit());
    time rise_time,last_rise=0;
    task check_width(input integer width);
        begin
            @(posedge pwm);rise_time=$time;
            if(last_rise && rise_time-last_rise!=20000000) $fatal(1,"PWM period changed on mode transition");
            last_rise=rise_time;
            @(negedge pwm);
            if($time-rise_time!=width*1000) $fatal(1,"PWM width %0t expected %0d us",$time-rise_time,width);
        end
    endtask
    initial begin
        repeat(4) @(negedge clk);rst_n=1;
        check_width(1500);
        @(negedge clk);enable=1;valid=1;
        @(negedge clk);valid=0;
        fork
            check_width(1505);
            begin
                @(posedge pwm);#200000;enable=0;
                repeat(3) @(negedge clk);
                if(pulse!=1500 || dut.pending_valid) $fatal(1,"Did not park/flush target on mode exit");
            end
        join
        check_width(1500);
        @(negedge clk);enable=1;
        check_width(1500);
        $display("PASS servo_park: startup center, exit during high pulse, intact period/width, center next frame, reentry without stale target");
        $finish;
    end
    initial begin #120000000;$fatal(1,"timeout");end
endmodule
