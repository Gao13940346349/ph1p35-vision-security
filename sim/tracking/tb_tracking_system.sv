`timescale 1ns/1ps
module tb_tracking_system;
    reg sys=0,cam=0,cam_running=1,rst=0,enable=0,key_n=1,send=0,bad=0;
    always #500 sys=~sys;
    always #137 if(cam_running) cam=~cam;
    reg [15:0] fid=0,capid=0;
    reg [422:0] boxes=0;
    wire pwm,publish;
    wire [15:0] pulse;
    wire [495:0] frame_result;
    integer publications=0;
    tracking_system #(.CLK_HZ(1000000),.KEY_DEBOUNCE_BITS(3),
        .CONTROL_US(4000),.WATCHDOG_US(10000)) dut(
        .sys_clk(sys),.camera_clk(cam),.rst_n(rst),.enable(enable),.next_button_n(key_n),
        .result_strobe(send),.result_bad(bad),.result_id(fid),.capture_id(capid),.boxes(boxes),
        .pan_pwm(pwm),.pulse_us(pulse),.frame_publish(publish),.frame_result(frame_result));
    always @(negedge cam) if(publish) begin
        publications++;
        if(frame_result[495:480]!==dut.source_id || frame_result[422:0]!==boxes)
            $fatal(1,"display metadata not an atomic detector-frame snapshot");
    end
    function automatic [46:0] box(input integer c,s,x);
        box={2'(c),2'(s),1'b1,11'(x-40),11'(x+40),10'd320,10'd400};
    endfunction
    task automatic result(input integer n,input integer capture,input bit quality_bad=0);
        @(negedge cam);fid=16'(n);capid=16'(capture);bad=quality_bad;send=1;
        @(negedge cam);send=0;bad=0;
        repeat(90)@(negedge cam);repeat(12)@(negedge sys);
    endtask
    task automatic correction(input integer expected);
        @(negedge sys);while(!dut.u_pan.control_tick)@(negedge sys);
        @(posedge sys);#1;
        if(pulse!==16'(expected))$fatal(1,"integrated servo pulse%0d expected%0d",pulse,expected);
    endtask
    task automatic press;
        @(negedge sys);key_n=0;repeat(25)@(negedge sys);
        key_n=1;repeat(25)@(negedge sys);
    endtask
    integer saved_publications;
    initial begin
        repeat(5)@(negedge sys);rst=1;
        boxes[0+:47]=box(1,2,800);boxes[47+:47]=box(3,1,500);
        result(65533,65534);
        if(publish || publications!=0)$fatal(1,"disabled mode published tracking/moved servo");
        capid=65534;enable=1;repeat(12)@(negedge sys);
        if(publications!=0)$fatal(1,"enable event was incorrectly tagged as a detector frame");
        result(65533,65534);
        if(publications!=0)$fatal(1,"pre-enable detector work crossed source barrier");
        result(65534,65534);result(65535,65535);
        if(dut.u_pan.acquired_once)$fatal(1,"servo started before target confirmation");
        result(0,0);
        if(dut.track_state!=2 || dut.track_id!=1 || !dut.u_pan.acquired_once || dut.u_pan.last_frame!=0)
            $fatal(1,"preferred red circle / wrapped source / servo handoff failed");
        correction(1505);correction(1505);
        result(1,1);correction(1510);
        // Build an old locked result just before camera publication, then stop
        // CSI. KEY1 must invalidate pending control in the independent domain.
        @(negedge cam);fid=2;capid=2;send=1;
        @(negedge cam);send=0;
        wait(dut.u_tracker.machine==5);
        @(negedge cam);cam_running=0;
        saved_publications=publications;press();
        if(dut.u_pan.pending_valid || dut.u_pan.control_active || dut.requested_epoch!=1)
            $fatal(1,"KEY1 did not freeze old target while camera stopped");
        correction(1510);
        cam_running=1;repeat(90)@(negedge cam);repeat(15)@(negedge sys);
        if(dut.u_pan.pending_valid)$fatal(1,"old in-flight target replayed after switching epoch");
        // Cycling wraps from the rightmost red circle to the left blue square.
        result(3,3);
        if(dut.track_state!=1 || dut.selected_box[46:43]!={2'd3,2'd1})$fatal(1,"KEY1 failed to cycle visible category");
        result(4,4);result(5,5);correction(1506);
        // Lost target holds its last PWM command; no scanning or continued step.
        boxes=0;result(6,6);
        if(dut.u_pan.pending_valid || dut.track_state!=3)$fatal(1,"loss did not stop correction");
        correction(1506);
        @(negedge cam);cam_running=0;
        #15000000;
        if(dut.u_pan.measurement_fresh || dut.u_pan.pending_valid || pulse!=1506 || !dut.u_pan.acquired_once)
            $fatal(1,"stopped-camera watchdog did not hold final command");
        // Disable/re-enable while CSI remains stopped, then change source
        // sequence radically. No old lock or mailbox message can survive.
        @(negedge sys);enable=0;
        repeat(8)@(negedge sys);enable=1;repeat(30)@(negedge sys);
        if(dut.u_pan.acquired_once || dut.u_pan.pending_valid || pulse!=1500)$fatal(1,"restart replayed prior session");
        boxes[0+:47]=box(1,2,800);capid=50000;cam_running=1;
        repeat(15)@(negedge sys);
        result(50000,50000);result(50001,50001);result(50002,50002);
        correction(1505);
        if(dut.u_pan.last_frame!=50002)$fatal(1,"restart could not establish fresh independent frame sequence");
        $display("PASS tracking_system: auto mode, preferred acquisition, confirmation, wrap, frame metadata, KEY1 epoch with stopped CSI, category cycle, loss/watchdog hold, mode flush and restart");
        $finish;
    end
    initial begin #120000000;$fatal(1,"tracking system timeout");end
endmodule
