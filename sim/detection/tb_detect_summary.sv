`timescale 1ns/1ps
module tb_detect_summary;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0,de=0,user=0,enable=1;
    reg [11:0] x=0,y=0;
    reg [422:0] boxes=0;
    wire hit; wire [23:0] colour;
    detect_summary_overlay dut(.clk(clk),.rst_n(rst_n),.de(de),.user(user),
        .enable(enable),.x(x),.y(y),.boxes(boxes),.hit(hit),.colour(colour));

    task automatic put(input integer slot,c,s,input bit valid);
        boxes[slot*47 +:47]={2'(c),2'(s),valid,11'd10,11'd80,10'd20,10'd90};
    endtask
    task automatic frame_start;
        @(negedge clk); user=1; de=1; x=0; y=0;
        @(negedge clk); user=0; de=0;
        repeat(12) @(negedge clk);
    endtask
    task automatic sample(input integer px,py,input bit expected_hit,
                          input [23:0] expected_colour);
        @(negedge clk); x=px; y=py; de=1;
        repeat(2) @(posedge clk); #1;
        if(hit!==expected_hit || (expected_hit && colour!==expected_colour))
            $fatal(1,"summary pixel %0d,%0d: hit=%b colour=%h expected=%b/%h",
                   px,py,hit,colour,expected_hit,expected_colour);
    endtask

    initial begin
        repeat(5) @(negedge clk); rst_n=1;
        put(0,1,0,1); put(1,1,0,1); put(2,1,2,1);
        put(3,2,1,1); put(4,2,1,1); put(5,2,1,1);
        put(6,3,2,1); put(7,1,0,0); put(8,3,3,1);
        frame_start();
        if(dut.shown_total!==7 || dut.shown_count[0]!==2 ||
           dut.shown_count[2]!==1 || dut.shown_count[4]!==3 ||
           dut.shown_count[8]!==1 || dut.shown_count[6]!==0)
            $fatal(1,"summary category counts incorrect");
        if(dut.panel_char(3'd0,5'd9,dut.shown_total,dut.shown)!="7" ||
           dut.panel_char(3'd2,5'd8,dut.shown_total,dut.shown)!="2" ||
           dut.panel_char(3'd3,5'd12,dut.shown_total,dut.shown)!="3")
            $fatal(1,"summary count text incorrect");
        sample(10,10,1,24'h102030);
        sample(18,20,1,24'hffffff); // The D in DETECTED.
        sample(0,0,0,0);
        enable=0; sample(10,10,0,0);
        enable=1; boxes=0; frame_start();
        if(dut.shown_total!==0 || dut.shown_count[0]!==0 ||
           dut.panel_char(3'd0,5'd9,dut.shown_total,dut.shown)!="0")
            $fatal(1,"summary did not clear on empty frame");
        $display("PASS detect_summary: nine categories, total, invalid slots, glyphs, panel gating, empty frame");
        $finish;
    end
    initial begin #1000000; $fatal(1,"summary timeout"); end
endmodule
