// B8 RGB block codec: 8 horizontal pixels -> 12 bytes, fixed 2:1 compression.
// Layout: min R,G,B; max R,G,B; 8 MSB-first {R2,G2,B2} palette indices.
// Each channel palette is lo, round((2lo+hi)/3), round((lo+2hi)/3), hi.
// No multipliers, dividers, line buffers or frame buffers are required.
module rgb_block_encoder (
    input wire clk, rst_n,
    input wire [127:0] in_data,
    input wire in_valid, in_empty,
    output reg in_re,
    input wire out_ready,
    output reg out_we,
    output reg [127:0] out_data
);
    localparam GET=0, WAIT_WORD=1, INIT=2, SCAN=3, INDEX=4, EMIT=5;
    reg [2:0] state;
    reg [1:0] word_count;
    reg [383:0] pixels;
    reg half_block;
    reg [2:0] pixel_count;
    reg [7:0] lo_r,lo_g,lo_b,hi_r,hi_g,hi_b;
    reg [47:0] indices;
    reg [95:0] block_data;
    reg [3:0] emit_count, pack_count;
    reg [127:0] packed_bytes;
    wire [7:0] r=pixels[383:376], g=pixels[375:368], b=pixels[367:360];
    function [1:0] quantize;
        input [7:0] v,lo,hi;
        reg [10:0] v6,t0,t1,t2;
        begin
            v6=({3'd0,v}<<2)+({3'd0,v}<<1);
            t0=({3'd0,lo}<<2)+{3'd0,lo}+{3'd0,hi};
            t1=(({3'd0,lo}+{3'd0,hi})<<1)+({3'd0,lo}+{3'd0,hi});
            t2={3'd0,lo}+({3'd0,hi}<<2)+{3'd0,hi};
            quantize = v6<t0 ? 0 : v6<t1 ? 1 : v6<t2 ? 2 : 3;
        end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=GET; word_count<=0; pixels<=0; half_block<=0;
            pixel_count<=0; indices<=0; block_data<=0;
            lo_r<=0; lo_g<=0; lo_b<=0; hi_r<=0; hi_g<=0; hi_b<=0;
            emit_count<=0; pack_count<=0; packed_bytes<=0;
            in_re<=0; out_we<=0; out_data<=0;
        end else begin
            in_re<=0; out_we<=0;
            case(state)
                GET: if(!in_empty) begin in_re<=1; state<=WAIT_WORD; end
                WAIT_WORD: if(in_valid) begin
                    pixels<={pixels[255:0],in_data};
                    if(word_count==2) begin word_count<=0; half_block<=0; state<=INIT; end
                    else begin word_count<=word_count+1'b1; state<=GET; end
                end
                INIT: begin
                    lo_r<=255; lo_g<=255; lo_b<=255;
                    hi_r<=0; hi_g<=0; hi_b<=0;
                    pixel_count<=0; indices<=0; state<=SCAN;
                end
                SCAN: begin
                    if(r<lo_r) lo_r<=r; if(r>hi_r) hi_r<=r;
                    if(g<lo_g) lo_g<=g; if(g>hi_g) hi_g<=g;
                    if(b<lo_b) lo_b<=b; if(b>hi_b) hi_b<=b;
                    // Rotate only the current eight-pixel half. The next
                    // half stays intact while extrema and indices are found.
                    pixels<={pixels[359:192],pixels[383:360],pixels[191:0]};
                    pixel_count<=pixel_count+1'b1;
                    if(pixel_count==7) state<=INDEX;
                end
                INDEX: begin
                    indices<={indices[41:0],quantize(r,lo_r,hi_r),quantize(g,lo_g,hi_g),quantize(b,lo_b,hi_b)};
                    pixels<={pixels[359:192],pixels[383:360],pixels[191:0]};
                    pixel_count<=pixel_count+1'b1;
                    if(pixel_count==7) begin
                        block_data<={lo_r,lo_g,lo_b,hi_r,hi_g,hi_b,indices[41:0],
                            quantize(r,lo_r,hi_r),quantize(g,lo_g,hi_g),quantize(b,lo_b,hi_b)};
                        emit_count<=0; state<=EMIT;
                    end
                end
                EMIT: if(out_ready) begin
                    packed_bytes<={packed_bytes[119:0],block_data[95:88]};
                    block_data<={block_data[87:0],8'd0};
                    pack_count<=pack_count+1'b1;
                    if(pack_count==15) begin
                        out_we<=1; out_data<={packed_bytes[119:0],block_data[95:88]};
                    end
                    if(emit_count==11) begin
                        if(!half_block) begin
                            pixels<={pixels[191:0],192'd0}; half_block<=1; state<=INIT;
                        end else state<=GET;
                    end else emit_count<=emit_count+1'b1;
                end
                default: state<=GET;
            endcase
        end
    end
endmodule
