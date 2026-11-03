`timescale 1ns/1ps
module tb_tracking_dual;
    reg sys=0,cam=0,cam_running=1,rst=0,enable=0,key_n=1,send=0,bad=0;
    always #500 sys=~sys;
    always #137 if(cam_running) cam=~cam;
    reg [15:0] fid=0,capid=0;
    reg [422:0] boxes=0;
    wire pwm,tilt_pwm,publish;
    wire [15:0] pulse,tilt_pulse;
    wire [495:0] frame_result;
    integer publications=0;
    tracking_system #(.CLK_HZ(1000000),.KEY_DEBOUNCE_BITS(3),
        .CONTROL_US(1000),.WATCHDOG_US(10000),
        .MIN_US(834),.MAX_US(2166),.DIRECTION(-1),.TILT_SUPPORT(1),
        .TILT_MIN_US(1352),.TILT_MAX_US(1648)) dut(
        .sys_clk(sys),.camera_clk(cam),.rst_n(rst),.enable(enable),.next_button_n(key_n),
        .result_strobe(send),.result_bad(bad),.result_id(fid),.capture_id(capid),.boxes(boxes),
        .pan_pwm(pwm),.pulse_us(pulse),.tilt_pwm(tilt_pwm),.tilt_pulse_us(tilt_pulse),.frame_publish(publish),.frame_result(frame_result));
    always @(negedge cam) if(publish) begin
        publications++;
        if(frame_result[495:480]!==dut.source_id || frame_result[422:0]!==boxes)
            $fatal(1,"display metadata not an atomic detector-frame snapshot");
    end
    function automatic [46:0] box(input integer c,s,x,y);
        box={2'(c),2'(s),1'b1,11'(x-40),11'(x+40),10'(y-40),10'(y+40)};
    endfunction
    task automatic result(input integer n,input integer capture,input bit quality_bad=0);
        @(negedge cam);fid=16'(n);capid=16'(capture);bad=quality_bad;send=1;
        @(negedge cam);send=0;bad=0;
        repeat(90)@(negedge cam);repeat(12)@(negedge sys);
    endtask
    task automatic correction(input integer expected,input integer expected_tilt);
        @(negedge sys);while(!dut.u_pan.control_tick)@(negedge sys);
        @(posedge sys);#1;
        if(pulse!==16'(expected))$fatal(1,"integrated servo pulse%0d expected%0d",pulse,expected);
        if(tilt_pulse!==16'(expected_tilt))$fatal(1,"integrated tilt pulse%0d expected%0d",tilt_pulse,expected_tilt);
        if(dut.u_pan.last_frame!==dut.g_tilt.u_tilt.last_frame ||
           dut.u_pan.pending_valid!==dut.g_tilt.u_tilt.pending_valid)
            $fatal(1,"axes disagree on atomic source frame/validity");
    endtask
    task automatic press;
        @(negedge sys);key_n=0;repeat(25)@(negedge sys);
        key_n=1;repeat(25)@(negedge sys);
    endtask
    integer saved_publications;
    initial begin
        repeat(5)@(negedge sys);rst=1;
        boxes[0+:47]=box(1,2,800,520);boxes[47+:47]=box(3,1,500,200);
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
        correction(1495,1505);correction(1495,1505);
        result(1,1);correction(1490,1510);
        // Build an old locked result just before camera publication, then stop
        // CSI. KEY1 must invalidate pending control in the independent domain.
        @(negedge cam);fid=2;capid=2;send=1;
        @(negedge cam);send=0;
        wait(dut.u_tracker.machine==5);
        @(negedge cam);cam_running=0;
        saved_publications=publications;press();
        if(dut.u_pan.pending_valid || dut.g_tilt.u_tilt.pending_valid || dut.u_pan.control_active || dut.g_tilt.u_tilt.control_active || dut.requested_epoch!=1)
            $fatal(1,"KEY1 did not freeze old target while camera stopped");
        correction(1490,1510);
        cam_running=1;repeat(90)@(negedge cam);repeat(15)@(negedge sys);
        if(dut.u_pan.pending_valid || dut.g_tilt.u_tilt.pending_valid)$fatal(1,"old in-flight target replayed after switching epoch");
        // Cycling wraps from the rightmost red circle to the left blue square.
        result(3,3);
        if(dut.track_state!=1 || dut.selected_box[46:43]!={2'd3,2'd1})$fatal(1,"KEY1 failed to cycle visible category");
        result(4,4);result(5,5);correction(1494,1505);
        // Lost target holds its last PWM command; no scanning or continued step.
        boxes=0;result(6,6);
        if(dut.u_pan.pending_valid || dut.track_state!=3)$fatal(1,"loss did not stop correction");
        correction(1494,1505);
        @(negedge cam);cam_running=0;
        #15000000;
        if(dut.u_pan.measurement_fresh || dut.u_pan.pending_valid || pulse!=1494 || tilt_pulse!=1505 || dut.g_tilt.u_tilt.measurement_fresh || !dut.u_pan.acquired_once)
            $fatal(1,"stopped-camera watchdog did not hold final command");
        // Disable/re-enable while CSI remains stopped, then change source
        // sequence radically. No old lock or mailbox message can survive.
        @(negedge sys);enable=0;
        repeat(8)@(negedge sys);enable=1;repeat(30)@(negedge sys);
        if(dut.u_pan.acquired_once || dut.u_pan.pending_valid || dut.g_tilt.u_tilt.acquired_once || pulse!=1500 || tilt_pulse!=1500)$fatal(1,"restart replayed prior session");
        boxes[0+:47]=box(1,2,800,520);capid=50000;cam_running=1;
        repeat(15)@(negedge sys);
        result(50000,50000);result(50001,50001);result(50002,50002);
        correction(1495,1505);
        if(dut.u_pan.last_frame!=50002)$fatal(1,"restart could not establish fresh independent frame sequence");
        // Sustained fresh detections reach the widened pan endpoint and the
        // independent tilt endpoint, then saturate without arithmetic wrap.
        for(integer n=50003;n<50163;n++) begin
            result(n,n);
            @(negedge sys);while(!dut.u_pan.control_tick)@(negedge sys);
            @(posedge sys);#1;
            if(pulse<834 || pulse>2166 || tilt_pulse<1352 || tilt_pulse>1648)
                $fatal(1,"dual axis exceeded independently configured range");
        end
        if(pulse!=834 || tilt_pulse!=1648)$fatal(1,"dual endpoints not reached");
        // Switch to the opposite quadrant, exercise the other two endpoints.
        boxes[47+:47]=box(3,1,400,200);
        press();
        result(50163,50163);result(50164,50164);result(50165,50165);
        for(integer n=50166;n<50466;n++) begin
            result(n,n);
            @(negedge sys);while(!dut.u_pan.control_tick)@(negedge sys);
            @(posedge sys);#1;
            if(pulse<834 || pulse>2166 || tilt_pulse<1352 || tilt_pulse>1648)
                $fatal(1,"opposite quadrant exceeded axis range");
        end
        if(pulse!=2166 || tilt_pulse!=1352)$fatal(1,"opposite dual endpoints not reached");
        $display("PASS tracking_dual: reversed pan +/-90 deg, tilt +/-20 deg, independent axes, saturation, auto mode, preferred acquisition, confirmation, wrap, frame metadata, KEY1 epoch with stopped CSI, category cycle, loss/watchdog hold, mode flush and restart");
        $finish;
    end
    initial begin #950000000;$fatal(1,"tracking system timeout");end
endmodule
