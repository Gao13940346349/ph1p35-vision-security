`timescale 1ns/1ps
module tb_tracking_bridge;
    reg cam=0,ddr=0,hdmi=0,rst=0,send=0,ws=0,rs=0;
    always #4.444 cam=~cam;
    always #7.5 ddr=~ddr;
    always #6.734 hdmi=~hdmi;
    reg [495:0] result=0;
    reg [1:0] wp=0,rp=0;
    wire [479:0] shown;
    detect_frame_bridge #(.BOX_WIDTH(480)) dut(.rst_n(rst),.camera_clk(cam),.result_strobe(send),.result(result),
        .ddr_clk(ddr),.write_start(ws),.write_slot(wp),.read_start(rs),.read_slot(rp),
        .hdmi_clk(hdmi),.display_box(shown),.display_latency_valid(),.display_latency_overflow(),.display_latency_frames());
    function automatic [479:0] metadata(input integer n);
        reg [46:0] b;
        b={2'(1+n%3),2'(n%3),1'b1,11'(n*20),11'(n*20+80),10'd200,10'd280};
        metadata={2'(n%4),8'(n),b,376'd0,b};
    endfunction
    task automatic write_frame(input integer n);
        @(negedge ddr);wp=2'(n-1);ws=1;@(negedge ddr);ws=0;
    endtask
    task automatic publish(input integer n);
        @(negedge cam);result={16'(n),metadata(n)};send=1;
        @(negedge cam);send=0;repeat(12)@(negedge ddr);
    endtask
    task automatic display_frame(input integer slot,input [479:0] expected);
        @(negedge ddr);rp=2'(slot);rs=1;@(negedge ddr);rs=0;
        repeat(15)@(negedge hdmi);
        if(shown!==expected) $fatal(1,"tracking identity/category/state were not matched to DDR source frame");
    endtask
    initial begin
        repeat(5)@(negedge ddr);rst=1;
        write_frame(1);publish(1);display_frame(0,metadata(1));
        write_frame(2);display_frame(1,0);publish(2);
        if(shown!==0) $fatal(1,"late tracking result tore active display metadata");
        display_frame(1,metadata(2));
        write_frame(3);publish(3);write_frame(4);publish(4);
        display_frame(0,metadata(1));write_frame(5);display_frame(0,0);
        publish(1);display_frame(0,0);publish(5);display_frame(0,metadata(5));
        display_frame(3,metadata(4));
        $display("PASS tracking_bridge: 480-bit atomic candidate/selected/ID/state, late result, DDR slot reuse and stale-frame rejection");
        $finish;
    end
    initial begin #100000;$fatal(1,"tracking bridge timeout");end
endmodule
