`timescale 1ns/1ps
module tb_target_tracker;
    reg clk=0,rst_n=0,enable=0,switch_target=0,send=0,bad=0;
    reg [15:0] fid=0,capid=0;
    reg [422:0] boxes=0;
    wire publish,valid;
    wire [15:0] source,age;
    wire [422:0] published_boxes;
    wire [46:0] selected;
    wire [7:0] id,session;
    wire [1:0] state;
    integer publications=0;
    always #5 clk=~clk;
    target_tracker #(.LOST_HOLD_FRAMES(5)) dut(
        .clk(clk),.rst_n(rst_n),.enable(enable),.switch_target(switch_target),
        .result_strobe(send),.result_bad(bad),.result_id(fid),.capture_id(capid),.boxes(boxes),
        .publish(publish),.result_publish(),.source_id(source),.source_age(age),.publish_boxes(published_boxes),
        .selected_box(selected),.track_id(id),.session_id(session),
        .track_state(state),.measurement_valid(valid));
    always @(negedge clk) if(publish) publications++;
    function automatic [46:0] box(input integer colour,shape,x,y,w,h);
        box={2'(colour),2'(shape),1'b1,11'(x-w/2),11'(x+w/2),10'(y-h/2),10'(y+h/2)};
    endfunction
    function automatic integer selected_x();
        selected_x=(int'(selected[41:31])+int'(selected[30:20]))/2;
    endfunction
    task automatic result(input integer n,input integer capture,input [422:0] b,input bit quality_bad=0);
        @(negedge clk); fid=16'(n);capid=16'(capture);boxes=b;bad=quality_bad;send=1;
        @(negedge clk);send=0;bad=0;
        // Bounded scan: 5 clocks per match slot, 3 per cycle slot, plus commits.
        repeat(90)@(negedge clk);
    endtask
    task automatic key(input integer boundary);
        @(negedge clk);capid=16'(boundary);switch_target=1;
        @(negedge clk);switch_target=0;
        if(valid || state!=1)$fatal(1,"switch did not stop control immediately");
    endtask
    task automatic expect_state(input integer s,input bit v,input integer x);
        if(state!==2'(s)||valid!==v)$fatal(1,"state %0d valid %0b expected %0d %0b",state,valid,s,v);
        if(x>=0 && selected_x()!=x)$fatal(1,"selected x%0d expected %0d",selected_x(),x);
    endtask
    reg [422:0] scene;
    integer saved_id,saved_publications,saved_session;
    initial begin
        repeat(5)@(negedge clk);rst_n=1;
        repeat(2)@(negedge clk);capid=1;enable=1;
        repeat(3)@(negedge clk);
        expect_state(1,0,-1);
        scene=0;scene[0+:47]=box(1,2,500,360,80,80);
        scene[47+:47]=box(3,1,700,360,100,100);
        scene[94+:47]=box(2,0,1000,360,120,120);
        saved_publications=publications;
        result(0,1,scene);
        if(publications!=saved_publications)$fatal(1,"pre-enable frame was accepted");
        result(1,1,scene);expect_state(1,0,500);
        if(published_boxes!==scene)$fatal(1,"result snapshot differs");
        result(2,2,scene);expect_state(1,0,500);
        result(3,3,scene);expect_state(2,1,500);saved_id=id;
        // Area ordering changes; a closer-to-centre same-class newcomer must
        // not replace a geometrically consistent previously locked object.
        scene=0;scene[376+:47]=box(1,2,520,360,84,80);
        scene[0+:47]=box(1,2,640,360,140,140);
        scene[47+:47]=box(3,1,710,360,100,100);
        scene[94+:47]=box(2,0,1000,360,120,120);
        result(4,5,scene);expect_state(2,1,520);
        if(id!=saved_id||age!=1)$fatal(1,"slot reorder changed identity/age");
        // Two transient shape errors are accepted but the display category
        // remains circle. The third is invalid and the servo must stop.
        scene[376+:47]=box(1,1,530,360,84,80);
        result(5,5,scene);expect_state(2,1,530);
        if(selected[44:43]!=2)$fatal(1,"temporary class glitch changed target category");
        result(6,6,scene);expect_state(2,1,530);
        result(7,7,scene);expect_state(3,0,530);
        scene[376+:47]=box(1,2,540,360,84,80);
        result(8,8,scene);expect_state(2,1,540);
        if(id!=saved_id)$fatal(1,"short lost did not preserve identity");
        // Switch chooses the next object by position, including another class.
        key(9);result(8,9,scene);expect_state(1,0,540);
        result(9,9,scene);expect_state(1,0,640);
        if(id==saved_id)$fatal(1,"explicit switch failed to allocate identity");
        result(10,10,scene);result(11,11,scene);expect_state(2,1,640);
        key(12);result(12,12,scene);expect_state(1,0,710);
        result(13,13,scene);result(14,14,scene);expect_state(2,1,710);
        key(15);result(15,15,scene);expect_state(1,0,1000);
        result(16,16,scene);result(17,17,scene);expect_state(2,1,1000);
        key(18);result(18,18,scene);expect_state(1,0,540);
        result(19,19,scene);result(20,20,scene);expect_state(2,1,540);
        saved_id=id;
        // With one visible target, switching retains that object and lock.
        scene=0;scene[235+:47]=box(1,2,545,360,84,80);
        key(21);result(21,21,scene);expect_state(2,1,545);
        if(id!=saved_id)$fatal(1,"single target cycle created new identity");
        // Quality failures invalidate control without dropping identity.
        result(22,22,scene,1);expect_state(3,0,545);
        result(23,23,scene);expect_state(2,1,545);
        saved_publications=publications;
        result(23,23,scene);result(22,23,scene);result(24,28,scene);
        if(publications!=saved_publications)$fatal(1,"duplicate, older or stale result accepted");
        // All-zero and malformed descriptors cannot be acquired. A long loss
        // expires the old identity; next valid detection begins confirmation.
        result(24,24,0);expect_state(3,0,545);
        result(29,29,0);expect_state(1,0,-1);
        if(selected[42])$fatal(1,"expired target still valid");
        scene=0;scene[0+:47]=box(3,1,630,360,60,60);
        scene[47+:47]={2'd1,2'd2,1'b1,11'd700,11'd600,10'd20,10'd80};
        result(30,30,scene);expect_state(1,0,630);
        if(id==saved_id)$fatal(1,"new acquisition reused expired identity");
        result(31,31,scene);result(32,32,scene);expect_state(2,1,630);
        // A SOF crossing the short scan must update reported age and stop a
        // result that became stale between acceptance and publication.
        @(negedge clk);fid=33;capid=36;boxes=scene;send=1;
        @(negedge clk);send=0;
        repeat(3)@(negedge clk);capid=37;
        repeat(87)@(negedge clk);expect_state(3,0,630);
        if(age!=4)$fatal(1,"age not rechecked at publication");
        // Normal mode disables at once. Restart rejects an old detector result
        // completing after enable, then handles 16-bit source-frame wrap.
        @(negedge clk);enable=0;
        repeat(3)@(negedge clk);expect_state(0,0,-1);
        saved_session=session;
        @(negedge clk);capid=16'hfffe;enable=1;
        repeat(3)@(negedge clk);
        if(session==saved_session)$fatal(1,"new enable session unchanged");
        saved_publications=publications;
        result(65533,65534,scene);
        if(publications!=saved_publications)$fatal(1,"restart accepted pre-enable detector work");
        result(65534,65534,scene);expect_state(1,0,630);
        result(65535,65535,scene);result(0,0,scene);expect_state(2,1,630);
        if(source!=0)$fatal(1,"source wrap not accepted");
        @(negedge clk);enable=0;
        repeat(3)@(negedge clk);capid=100;enable=1;
        repeat(3)@(negedge clk);
        scene=0;scene[0+:47]=box(1,2,630,360,60,60);
        result(100,100,scene);expect_state(1,0,630);
        result(101,101,scene,1);
        result(102,102,scene);expect_state(1,0,630);
        result(103,103,scene);expect_state(1,0,630);
        result(104,104,scene);expect_state(2,1,630);
        @(negedge clk);enable=0;
        repeat(3)@(negedge clk);capid=110;enable=1;
        repeat(3)@(negedge clk);
        result(110,110,scene);expect_state(1,0,630);
        scene[0+:47]=box(1,1,630,360,60,60);
        result(111,111,scene);expect_state(3,0,630);
        scene[0+:47]=box(1,2,630,360,60,60);
        result(112,112,scene);expect_state(1,0,630);
        result(113,113,scene);expect_state(1,0,630);
        result(114,114,scene);expect_state(2,1,630);
        $display("PASS target_tracker: confirmation, category fallback, geometry identity, shape grace, loss/reacquisition, cycling, source age/order/barrier/wrap");
        $finish;
    end
    initial begin #1000000;$fatal(1,"watchdog");end
endmodule
