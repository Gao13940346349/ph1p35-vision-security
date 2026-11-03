`timescale 1ns/1ps
// Exact row/column scoreboard, including SOL before VALID and intra-line gaps.
module tb_line_buffer_blanking #(parameter WIDTH=1280);
    localparam GROUPS=WIDTH/4, HEIGHT=8;
    reg clk=0, rst_n=0, valid=0, sol=0;
    reg [31:0] din=0;
    wire [31:0] prev1,prev2;
    integer checks=0;
    always #5 clk=~clk;
    line_buffer_grp #(.DW(32),.MAX_GRPS(512)) dut(
        .clk(clk),.rst_n(rst_n),.vld(valid),.sol(sol),.din(din),
        .row_m1(prev1),.row_m2(prev2));
    function automatic [31:0] tag(input integer frame,input integer row,input integer col);
        tag=(frame<<24)|(row<<16)|col;
    endfunction
    initial begin
        repeat(5) @(negedge clk); rst_n=1;
        for(integer f=0;f<4;f=f+1) begin
            // No reset between frames: the frame-start pulse must re-anchor RAM.
            @(negedge clk); valid=0; sol=1;
            @(negedge clk); sol=0;
            repeat(37) @(negedge clk);
            for(integer y=0;y<HEIGHT;y=y+1) begin
                for(integer x=0;x<GROUPS;x=x+1) begin
                    @(negedge clk); valid=1;
                    // f=0 has continuous rows; f=1..3 issue SOL during blanking.
                    sol=(f==0 && x==0);
                    din=tag(f,y,x);
                    @(posedge clk);
                    if(dut.cur_addr!==x) $fatal(1,"Address mismatch f=%0d y=%0d x=%0d addr=%0d",f,y,x,dut.cur_addr);
                    #1;
                    if(y>=1 && prev1!==tag(f,y-1,x)) $fatal(1,"Previous row mismatch f=%0d y=%0d x=%0d",f,y,x);
                    if(y>=2 && prev2!==tag(f,y-2,x)) $fatal(1,"Previous two rows mismatch f=%0d y=%0d x=%0d",f,y,x);
                    checks=checks+1;
                    if(f==3 && x<GROUPS-1) begin
                        @(negedge clk); valid=0; sol=0; din=32'hdeadcafe;
                        repeat(x%3) @(negedge clk);
                    end
                end
                if(f!=0) begin
                    @(negedge clk); valid=0; sol=1; din=32'hbad0bad0;
                    @(negedge clk); sol=0;
                    repeat((y%2==0)?680:1000) @(negedge clk);
                end
            end
        end
        @(negedge clk); valid=0; sol=0;
        $display("PASS line_buffer_blanking width=%0d checks=%0d: exact address/previous rows across blanking and frames",WIDTH,checks);
        $finish;
    end
    initial begin #5000000; $fatal(1,"Line buffer test timeout"); end
endmodule
