`timescale 1ns/1ps
module tb_tracking_overlay;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,de=0,user=0,enable=1;
    reg [11:0] x=0,y=0;
    reg [46:0] tracked_box=0;
    reg [7:0] track_id=0;
    reg [1:0] track_state=0;
    wire hit;wire [23:0] colour;
    tracking_overlay dut(.clk(clk),.rst_n(rst_n),.de(de),.user(user),.enable(enable),
        .x(x),.y(y),.tracked_box(tracked_box),.track_id(track_id),.track_state(track_state),
        .hit(hit),.colour(colour));
    reg [422:0] boxes=0;
    wire summary_hit;wire [23:0] summary_colour;
    detect_summary_overlay #(.PANEL_X(176)) summary(.clk(clk),.rst_n(rst_n),.de(de),
        .user(user),.enable(enable),.x(x),.y(y),.boxes(boxes),.hit(summary_hit),.colour(summary_colour));

    task automatic frame_start;
        @(negedge clk);user=1;de=1;x=0;y=0;
        @(negedge clk);user=0;de=0;
        repeat(14) @(negedge clk);
    endtask
    task automatic sample(input integer px,py,input bit expected_hit,input [23:0] expected_colour);
        @(negedge clk);x=px;y=py;de=1;
        repeat(2) @(posedge clk);#1;
        if(hit!==expected_hit || (expected_hit && colour!==expected_colour))
            $fatal(1,"tracking pixel %0d,%0d hit=%b/%b colour=%h/%h",px,py,hit,expected_hit,colour,expected_colour);
    endtask
    task automatic summary_sample(input integer px,py,input bit expected_hit,input [23:0] expected_colour);
        @(negedge clk);x=px;y=py;de=1;
        repeat(2) @(posedge clk);#1;
        if(summary_hit!==expected_hit || (expected_hit && summary_colour!==expected_colour))
            $fatal(1,"relocated summary pixel %0d,%0d hit=%b/%b colour=%h/%h",px,py,summary_hit,expected_hit,summary_colour,expected_colour);
    endtask
    reg previous_hit;
    reg [23:0] previous_colour;
    integer i;
    initial begin
        repeat(4) @(negedge clk);rst_n=1;
        tracked_box={2'd1,2'd2,1'b1,11'd100,11'd220,10'd200,10'd320};
        track_id=8'd123;track_state=2'd2;boxes[46:0]=tracked_box;
        frame_start();
        if(dut.centre_x!==160 || dut.centre_y!==260 || dut.shown_x!==16'h0160 ||
           dut.shown_y!==16'h0260 || dut.shown_id!==12'h123)
            $fatal(1,"frame centre/identity BCD wrong");
        if(dut.panel_char(0,9)!="1" || dut.panel_char(0,10)!="2" ||
           dut.panel_char(0,11)!="3" || dut.panel_char(0,13)!="L" ||
           dut.panel_char(1,0)!="R" || dut.panel_char(1,7)!="C" ||
           dut.panel_char(2,3)!="1" || dut.panel_char(2,4)!="6" ||
           dut.panel_char(3,0)!="K" || dut.panel_char(3,1)!="5" ||
           dut.panel_char(3,4)!=":" || dut.panel_char(3,5)!="N" ||
           dut.panel_char(3,6)!="E" || dut.panel_char(3,7)!="X" ||
           dut.panel_char(3,8)!="T")
            $fatal(1,"current target text wrong");
        sample(512,8,1,24'h102030);
        sample(522,17,1,24'hfff200); // The T in TRACK, locked colour.
        sample(100,240,1,24'hfff200);
        sample(160,260,1,24'hfff200);
        sample(160,240,0,0);
        sample(640,360,1,24'h00e6ff);
        sample(0,0,0,0); // Logo area remains transparent.
        summary_sample(10,10,0,0);
        summary_sample(178,10,1,24'h102030);
        summary_sample(186,20,1,24'hffffff); // Relocated DETECTED glyph.

        // Raw incoming metadata changing mid-frame must not change the panel
        // or selected box until the next source-matched display frame starts.
        tracked_box={2'd3,2'd1,1'b1,11'd800,11'd1000,10'd400,10'd500};
        track_id=8'd255;track_state=2'd1;
        sample(100,240,1,24'hfff200);
        if(dut.panel_char(1,0)!="R" || dut.shown_id!==12'h123) $fatal(1,"metadata tore during frame");
        frame_start();
        if(dut.shown_id!==12'h255 || dut.shown_x!==16'h0900 || dut.shown_y!==16'h0450 ||
           dut.panel_char(1,0)!="B" || dut.panel_char(1,7)!="S" || dut.panel_char(0,13)!="A")
            $fatal(1,"target switch not committed at frame boundary");
        sample(800,450,0,0); // Acquiring target is not shown as locked.
        track_state=2'd3;frame_start();
        sample(522,17,1,24'hffb040);
        sample(900,450,0,0); // Lost measurement cannot resemble an active lock.
        tracked_box=0;track_id=0;track_state=0;frame_start();
        if(dut.panel_char(1,0)!="N" || dut.panel_char(2,2)!=" ") $fatal(1,"empty target is not explicit");
        enable=0;sample(512,8,0,0);sample(640,360,0,0);

        // Stream alternating on/off points to verify two pipeline stages,
        // rather than testing only constant pixels.
        enable=1;tracked_box={2'd1,2'd0,1'b1,11'd100,11'd220,10'd200,10'd320};
        track_state=2'd2;frame_start();
        @(negedge clk);de=0;
        repeat(3) @(posedge clk);
        previous_hit=0;previous_colour=0;
        for(i=0;i<12;i=i+1) begin
            @(negedge clk);x=i[0] ? 12'd160 : 12'd100;y=12'd240;de=1;
            @(posedge clk);#1;
            if(hit!==previous_hit || (previous_hit && colour!==previous_colour))
                $fatal(1,"stream pixel/output stages misaligned at %0d",i);
            previous_hit=!i[0];previous_colour=24'hfff200;
        end
        @(negedge clk);de=0;
        @(posedge clk);#1;if(hit!==previous_hit) $fatal(1,"stream tail was dropped");
        @(posedge clk);#1;if(hit!==0) $fatal(1,"DE gating tail wrong");
        $display("PASS tracking_overlay: frame-held ID/class/decimal centre, switch, states, selected marker, Logo transparency, relocated summary, two-stage stream alignment");
        $finish;
    end
    initial begin #1000000;$fatal(1,"tracking overlay timeout");end
endmodule
