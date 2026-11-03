`timescale 1ns/1ps
module tb_tracking_mailbox;
    reg src=0,dst=0,src_running=1,dst_running=1,rst=0,send=0;
    always #5 if(src_running) src=~src;
    always #17 if(dst_running) dst=~dst;
    reg [15:0] data=0;
    wire receive;wire [15:0] output_data;
    integer count=0;
    reg [15:0] seen[0:7];
    tracking_mailbox #(.WIDTH(16)) dut(.arst_n(rst),.src_clk(src),.send(send),.data_in(data),
        .dst_clk(dst),.receive(receive),.data_out(output_data));
    always @(negedge dst) if(receive) begin seen[count]=output_data;count++;end
    task automatic put(input integer value);
        @(negedge src);data=16'(value);send=1;
        @(negedge src);send=0;
    endtask
    initial begin
        #80;rst=1;repeat(6)@(negedge src);
        @(negedge dst);dst_running=0;
        put(1);put(2);put(3);
        if(dut.held!=1 || dut.latest!=3 || !dut.pending) $fatal(1,"busy mailbox corrupted held/latest payload");
        #80;dst_running=1;
        repeat(20)@(negedge dst);
        if(count!=2 || seen[0]!=1 || seen[1]!=3) $fatal(1,"latest pending was not delivered, count%0d",count);
        // Stop the source with an in-flight transfer, then flush asynchronously.
        @(negedge dst);dst_running=0;
        put(4);put(5);
        @(negedge src);src_running=0;rst=0;
        #20;
        if(dut.request || dut.pending || receive || output_data) $fatal(1,"mode-off did not flush without clocks");
        rst=1;dst_running=1;
        repeat(12)@(negedge dst);
        if(count!=2) $fatal(1,"disabled-session payload replayed while camera stopped");
        src_running=1;repeat(6)@(negedge src);put(6);
        repeat(12)@(negedge dst);
        if(count!=3 || seen[2]!=6) $fatal(1,"re-enabled session delivery failed");
        $display("PASS tracking_mailbox: bundled data, latest pending, asynchronous flush with stopped source, no old-session replay");
        $finish;
    end
    initial begin #100000;$fatal(1,"mailbox timeout");end
endmodule
