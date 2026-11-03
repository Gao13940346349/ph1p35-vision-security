`timescale 1ns/1ps
module tb_security_tracking;
    reg sys=0,cam=0,cam_running=1;
    always #7 sys=~sys;
    always #5 if(cam_running) cam=~cam;
    reg rst_n=0,enable=0,key_n=1,strobe=0,bad=0,application=1,track=0;
    reg [46:0] seed=0;
    reg [422:0] boxes=0;
    reg [15:0] id=0,capture=0;
    wire [15:0] pan,tilt;
    wire publish;
    wire [495:0] result;
    integer publications=0;
    tracking_system #(.SECURITY_SUPPORT(1),.CLK_HZ(1000000),.KEY_DEBOUNCE_BITS(2),
        .TILT_SUPPORT(1),.CONTROL_US(50),.WATCHDOG_US(1000)) dut(
        .sys_clk(sys),.camera_clk(cam),.rst_n(rst_n),.enable(enable),.next_button_n(key_n),
        .result_strobe(strobe),.result_bad(bad),.result_id(id),.capture_id(capture),.boxes(boxes),
        .app_mode(application),.app_track_enable(track),.app_seed(seed),
        .pan_pwm(),.pulse_us(pan),.tilt_pwm(),.tilt_pulse_us(tilt),.frame_publish(publish),.frame_result(result));
    function automatic [46:0] box(input integer c,s,x,y);
        box={2'(c),2'(s),1'b1,11'(x-30),11'(x+30),10'(y-30),10'(y+30)};
    endfunction
    task automatic check(input bit ok,input string msg);
        if(!ok) $fatal(1,"%s state=%d box=%h pan=%d tilt=%d",msg,dut.track_state,dut.selected_box,pan,tilt);
    endtask
    always @(negedge cam) if(publish) begin
        publications++;
        if(result[422:0]!==boxes) $fatal(1,"publication lost full source detections");
    end
    task automatic frame;
        @(negedge cam);id=id+1'b1;capture=id;strobe=1;
        @(negedge cam);strobe=0;repeat(100) @(negedge cam);repeat(30) @(negedge sys);
    endtask
    task automatic press;
        @(negedge sys);key_n=0;repeat(20) @(negedge sys);key_n=1;repeat(20) @(negedge sys);
    endtask
    initial begin
        repeat(5) @(negedge sys);rst_n=1;enable=1;repeat(10) @(negedge sys);
        boxes[0+:47]=box(1,2,900,440);boxes[47+:47]=box(3,0,1050,470);
        repeat(4) frame();
        check(publications==4 && pan==1500 && tilt==1500,"fixed monitoring publishes boxes and parks both axes");
        check(dut.selected_box==0 && dut.track_state==0,"monitoring has no generic selected target");
        // Event chooses blue triangle, even though normal tracker prefers red circle.
        @(negedge cam);seed=boxes[47+:47];track=1;
        repeat(10) @(negedge cam);frame();
        check(dut.selected_box[46:43]=={2'd3,2'd0} && dut.track_state==2,"application overrides normal preferred acquisition");
        check(pan>1500 && tilt>1500,"both axes follow seeded incident");
        press();frame();check(dut.selected_box[46:43]=={2'd3,2'd0},"application K5 never cycles target");
        // Drop seeded category for more than normal lost hold: never acquire red.
        boxes[47+:47]=0;repeat(20) frame();
        check(dut.track_state==3 && dut.selected_box[46:43]=={2'd3,2'd0},"loss retains incident identity without reacquisition");
        boxes[47+:47]=box(3,0,1030,450);frame();
        check(dut.track_state==2 && dut.selected_box[46:43]=={2'd3,2'd0},"incident resumes same nearby category after loss");
        // Cut a sample in flight while acknowledging, before servo mailbox consumes.
        @(negedge cam);id=id+1'b1;capture=id;strobe=1;
        @(negedge cam);strobe=0;wait(dut.u_tracker.machine==5);
        @(negedge cam);track=0;
        repeat(10) @(negedge sys);
        check(pan==1500 && tilt==1500 && !dut.u_pan.pending_valid,"ACK flushes pending servo sample and parks immediately");
        repeat(100) @(negedge cam);repeat(30) @(negedge sys);
        check(pan==1500 && tilt==1500 && !dut.u_pan.pending_valid,"old camera publication cannot replay after ACK");
        frame();check(dut.selected_box==0 && dut.track_state==0,"ACK returned to fixed-monitor publication");
        // New incident can use a different category with fresh seed.
        @(negedge cam);seed=boxes[0+:47];track=1;repeat(10) @(negedge cam);frame();
        check(dut.selected_box[46:43]=={2'd1,2'd2} && dut.track_state==2,"new event starts fresh incident identity");
        // Mode-off must park even if CSI stops entirely.
        @(negedge cam);cam_running=0;@(negedge sys);enable=0;
        repeat(10) @(negedge sys);
        check(pan==1500 && tilt==1500 && !dut.u_pan.pending_valid,"stopped CSI cannot block mode-off parking");
        $display("PASS security tracking: fixed monitoring, seeded priority, dual-axis steering, NEXT ownership, loss, ACK mailbox flush, new event, stopped camera exit");
        $finish;
    end
    initial begin #2000000;$fatal(1,"security tracking timeout");end
endmodule
