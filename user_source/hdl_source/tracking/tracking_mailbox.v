// Bundled-data CDC with a one-message latest-value pending slot. Mode-off
// asynchronously flushes both sides; each clock synchronizes reset release.
module tracking_mailbox #(parameter integer WIDTH=90)(
    input wire arst_n, src_clk, send,
    input wire [WIDTH-1:0] data_in,
    input wire dst_clk,
    output reg receive,
    output reg [WIDTH-1:0] data_out
);
    reg [1:0] src_reset, dst_reset;
    always @(posedge src_clk or negedge arst_n)
        if(!arst_n) src_reset<=0; else src_reset<={src_reset[0],1'b1};
    always @(posedge dst_clk or negedge arst_n)
        if(!arst_n) dst_reset<=0; else dst_reset<={dst_reset[0],1'b1};
    reg request,ack,ack0,ack1,req0,req1,pending;
    reg [WIDTH-1:0] held,latest;
    wire ready=request==ack1;
    always @(posedge src_clk or negedge src_reset[1]) begin
        if(!src_reset[1]) begin
            request<=0; ack0<=0; ack1<=0; held<=0; latest<=0; pending<=0;
        end else begin
            ack0<=ack; ack1<=ack0;
            if(send) begin
                if(ready) begin held<=data_in; request<=!request; pending<=0; end
                else begin latest<=data_in; pending<=1; end
            end else if(pending && ready) begin
                held<=latest; request<=!request; pending<=0;
            end
        end
    end
    always @(posedge dst_clk or negedge dst_reset[1]) begin
        if(!dst_reset[1]) begin ack<=0;req0<=0;req1<=0;receive<=0;data_out<=0; end
        else begin
            req0<=request;req1<=req0;receive<=0;
            if(req1!=ack) begin data_out<=held;receive<=1;ack<=req1; end
        end
    end
endmodule
