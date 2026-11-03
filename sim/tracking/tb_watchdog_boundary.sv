`timescale 1ns/1ps
module tb_watchdog_boundary;
    reg clk=0,rst=0,enable=0,valid=0;
    reg [15:0] fid=1;
    always #500 clk=~clk;
    wire [15:0] pulse;
    wire fresh;
    pan_servo_control #(.CLK_HZ(1000000),.CONTROL_US(4),.WATCHDOG_US(4)) dut(
        .clk(clk),.rst_n(rst),.enable(enable),.sample_valid(valid),.sample_locked(1'b1),
        .sample_x(11'd1000),.sample_frame_id(fid),.pwm(),.pulse_us(pulse),
        .measurement_fresh(fresh),.control_active(),.at_limit());
    initial begin
        repeat(5)@(negedge clk);rst=1;enable=1;
        while(!dut.control_tick)@(negedge clk);
        valid=1;@(negedge clk);valid=0;
        if(!fresh || pulse!=1505 || dut.pending_valid)$fatal(1,"new boundary measurement was not consumed at tick");
        while(!dut.control_tick)@(negedge clk);
        @(posedge clk);#1;
        if(pulse!=1505 || fresh || dut.pending_valid)$fatal(1,"correction occurred exactly when measurement expired");
        repeat(3)begin
            @(negedge clk);while(!dut.control_tick)@(negedge clk);
            fid++;valid=1;@(posedge clk);#1;
            if(pulse!=1500+fid*5 || dut.pending_valid)$fatal(1,"phase-aligned measurements starved or replayed control");
            @(negedge clk);valid=0;
        end
        $display("PASS watchdog_boundary: coincident timeout freezes command; phase-aligned fresh detections advance once per tick");
        $finish;
    end
    initial begin #100000;$fatal(1,"watchdog boundary timeout");end
endmodule
