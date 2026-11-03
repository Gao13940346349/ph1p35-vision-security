`timescale 1ns/1ps
module tb_security_shared;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,enable=0,ack=0,strobe=0,bad=0;
    reg [15:0] id=0,capture=0;
    reg [422:0] boxes=0;
    wire snap,snap_bad,track;wire [15:0] snap_id,red,blue,event_id;
    wire [422:0] snap_boxes;wire [46:0] seed;wire [2:0] state,severity;
    wire [15:0] pan;
    tracking_system #(.SECURITY_SUPPORT(1),.CLK_HZ(1000000),.CONTROL_US(50),.WATCHDOG_US(1000)) tracker(
        .sys_clk(clk),.camera_clk(clk),.rst_n(rst_n),.enable(enable),.next_button_n(1'b1),
        .result_strobe(strobe),.result_bad(bad),.result_id(id),.capture_id(capture),.boxes(boxes),
        .app_mode(1'b1),.app_track_enable(track),.app_seed(seed),.pulse_us(pan),
        .snapshot_strobe(snap),.snapshot_bad(snap_bad),.snapshot_id(snap_id),.snapshot_boxes(snap_boxes));
    security_application #(.SHARED_BOXES(1),.CAMERA_CLK_HZ(1000),.SETTLE_MS(3)) dut(
        .clk(clk),.rst_n(rst_n),.enable(enable),.ack(ack),.result_strobe(snap),.result_bad(snap_bad),
        .result_id(snap_id),.capture_id(capture),.boxes(snap_boxes),
        .app_state(state),.severity(severity),.event_seed(seed),.track_enable(track),
        .red_count_bcd(red),.blue_count_bcd(blue),.event_source_id(event_id));
    function automatic [46:0] box(input integer c,s,x,y);
        box={2'(c),2'(s),1'b1,11'(x-30),11'(x+30),10'(y-30),10'(y+30)};
    endfunction
    task frame(input [422:0] b,input bit invalid=0);
        @(negedge clk);boxes=b;bad=invalid;id=id+1'b1;capture=id;strobe=1;
        @(negedge clk);strobe=0;
        // Raw detector bus is no longer owned; only tracker snapshot is stable.
        boxes='x;repeat(100)@(negedge clk);
    endtask
    reg [422:0] scene;
    integer c,s;
    initial begin
        repeat(4)@(negedge clk);rst_n=1;enable=1;repeat(15)@(negedge clk);
        scene=0;scene[8*47+:47]=box(1,2,980,400);
        frame(scene);frame(scene);
        if(track || red!=0)$fatal(1,"shared snapshot confirmed too early");
        frame(scene);
        if(!track || red!=1 || seed!==scene[8*47+:47] || event_id!=id)
            $fatal(1,"shared snapshot event/seed/source incorrect");
        frame(scene);if(tracker.track_state!=2)$fatal(1,"seeded tracker failed to lock");
        repeat(5)frame(scene);if(red!=1)$fatal(1,"moving camera recounted");
        @(negedge clk);ack=1;@(negedge clk);ack=0;repeat(15)@(negedge clk);
        frame(scene);if(pan!=1500 || state!=3)$fatal(1,"ACK failed to return/hold");
        repeat(6)frame(0);if(state!=1 || red!=1)$fatal(1,"shared clear/rearm failed");
        frame(scene);frame(scene,1);frame(scene);frame(scene);
        if(track)$fatal(1,"bad shared snapshot contributed to confirmation");
        frame(scene);if(!track || red!=2)$fatal(1,"shared reentry failed");
        // Every grade/type must survive the real accepted-snapshot -> policy ->
        // seeded-tracker loop; standalone policy acceptance alone is insufficient.
        for(c=1;c<=3;c=c+1)for(s=0;s<=2;s=s+1)begin
            @(negedge clk);rst_n=0;enable=0;ack=0;strobe=0;bad=0;boxes=0;id=0;capture=0;
            repeat(4)@(negedge clk);rst_n=1;enable=1;repeat(15)@(negedge clk);
            scene=0;scene[((c*3+s)%9)*47+:47]=box(c,s,980,400);
            repeat(3)frame(scene);
            if(c==2)begin
                if(track || severity!=0 || red!=0 || blue!=0)$fatal(1,"authorized category incorrectly triggered tracking shape=%d",s);
            end else begin
                if(!track || seed!==scene[((c*3+s)%9)*47+:47] || event_id!=id ||
                   red!=(c==1) || blue!=(c==3))$fatal(1,"grade/type shared event seed incorrect colour=%d shape=%d",c,s);
                frame(scene);
                if(tracker.track_state!=2)$fatal(1,"grade/type seeded tracker failed to lock colour=%d shape=%d",c,s);
            end
        end
        $display("PASS shared tracker/application snapshot: nine grade/type combinations, equipment/person/vehicle seeded locks, slot8, raw-bus overwrite, source identity, bad frame, ACK and rearm");
        $finish;
    end
    initial begin #1000000;$fatal(1,"shared integration timeout");end
endmodule
