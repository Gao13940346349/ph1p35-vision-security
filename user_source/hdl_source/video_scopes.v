// Display-domain scopes; observation only, never modifies ISP/DDR pixels.
// 32 bins/channel, one sample per 16 columns and 8 rows (7200 at 720p).
// A shared synchronous counter RAM serializes the four RMW operations.
module video_scopes #(
    parameter H_OFFSET=0, V_OFFSET=0, IMG_WIDTH=1280, IMG_HEIGHT=720,
    parameter PANEL_X=1008, PANEL_Y=192
)(
    input wire clk,rst_n,enable,
    input wire sample_de,
    input wire [11:0] sample_x,sample_y,
    input wire [23:0] rgb,
    input wire draw_de,
    input wire [11:0] draw_x,draw_y,
    output reg hit,
    output reg [23:0] colour
);
    localparam CLEAR=0, IDLE=1, READ_BIN=2, WRITE_BIN=3, SNAP_READ=4, SNAP_WRITE=5,
               Y_SCALE=6, Y_COMMIT=7, Y_LAST=8, Y_NEXT=9, Y_PIXEL=10, Y_PUBLISH=11;
    localparam [16:0] SAMPLE_COUNT=((IMG_WIDTH+15)/16)*((IMG_HEIGHT+7)/8);
    reg [3:0] state;
    reg [6:0] addr;
    reg [1:0] channel;
    reg [19:0] sample_bins;
    reg pending_end, armed, valid;
    (* ram_style="bram" *) reg [15:0] counts[0:127];
    // RGB addresses 0..95 hold log heights. Y addresses 256..511 hold the
    // lower/upper row of each of the 256 polyline columns. Both fit in the
    // same single 20 Kb ERAM; rendering needs only comparisons after its read.
    (* ram_style="bram" *) reg [13:0] heights[0:511];
    reg [15:0] count_q;
    reg [22:0] y_numerator;
    reg [16:0] y_remainder;
    reg [6:0] y_quotient,y_previous;
    reg [4:0] y_step;
    reg signed [10:0] y_slope,y_position,y_next_position;
    reg [2:0] y_fraction;
    wire [6:0] y_level=y_position[9:3],y_next_level=y_next_position[9:3];
    wire [6:0] y_low=y_level<y_next_level ? y_level : y_next_level;
    wire [6:0] y_high=y_level>y_next_level ? y_level : y_next_level;
    // Exactly one write port lets TD infer a simple-dual-port ERAM. Separate
    // indexed assignments to addr, addr-1 and 127 prevented that inference.
    wire height_write=(state==SNAP_WRITE && addr<96) ||
                      state==Y_PIXEL || state==Y_LAST;
    wire [4:0] y_segment=addr[4:0]-5'd1;
    wire [8:0] height_write_addr=state==Y_LAST ? {1'b1,5'd31,y_fraction} :
                                state==Y_PIXEL ? {1'b1,y_segment,y_fraction} : {2'd0,addr};
    wire [13:0] height_write_data=state==SNAP_WRITE ? {7'd0,height_of(count_q)} :
                                  state==Y_LAST ? {y_previous,y_previous} :
                                  {y_high,y_low};
    wire [16:0] y_trial={y_remainder[15:0],y_numerator[22]};
    wire y_subtract=y_trial>=SAMPLE_COUNT;
    wire [11:0] sx=sample_x-H_OFFSET, sy=sample_y-V_OFFSET;
    wire inside_image=sample_de && sample_x>=H_OFFSET && sample_y>=V_OFFSET &&
                      sx<IMG_WIDTH && sy<IMG_HEIGHT;
    // Display luma approximation (R+2G+B)/4, explicitly not sensor RAW luma.
    wire [9:0] luma_sum={2'b0,rgb[23:16]}+{1'b0,rgb[15:8],1'b0}+{2'b0,rgb[7:0]};
    // Fixed log2-like RGB scale shared across frames: 6 pixels/octave.
    // Zero stays zero; 1 -> 6, 2 -> 12, 4 -> 18, ... No per-frame auto-scaling.
    function [6:0] height_of;
        input [15:0] n;
        integer k;
        reg [6:0] h;
        begin h=0; for(k=0;k<16;k=k+1) if(n[k]) h=(k==15)?95:(k+1)*6; height_of=h; end
    endfunction
    always @(posedge clk) begin
        count_q<=counts[addr];
        if(rst_n) begin
            if(state==CLEAR || state==SNAP_WRITE) counts[addr]<=0;
            else if(state==WRITE_BIN) counts[addr]<=(&count_q) ? count_q : count_q+1'b1;
            if(height_write) heights[height_write_addr]<=height_write_data;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin state<=CLEAR;addr<=0;channel<=0;sample_bins<=0;
            pending_end<=0;armed<=0;valid<=0;
            y_numerator<=0;y_remainder<=0;y_quotient<=0;y_previous<=0;y_step<=0;
            y_slope<=0;y_position<=0;y_next_position<=0;y_fraction<=0;end
        else begin
            if(state==IDLE && inside_image && sx==0 && sy==0) armed<=1;
            if(inside_image && armed && sx==IMG_WIDTH-1 && sy==IMG_HEIGHT-1) pending_end<=1;
            case(state)
                CLEAR: if(addr==127) begin addr<=0;state<=IDLE;end else addr<=addr+1'b1;
                IDLE: if(pending_end) begin pending_end<=0;armed<=0;addr<=0;valid<=0;state<=SNAP_READ;end
                    else if(inside_image && (armed || (sx==0 && sy==0)) && sx[3:0]==0 && sy[2:0]==0) begin
                        sample_bins<={luma_sum[9:5],rgb[7:3],rgb[15:11],rgb[23:19]};
                        addr<={2'd0,rgb[23:19]};channel<=0;state<=READ_BIN;
                    end
                READ_BIN: state<=WRITE_BIN;
                WRITE_BIN: if(channel==3) state<=IDLE;
                    else begin channel<=channel+1'b1;sample_bins<=sample_bins>>5;
                        addr<={channel+2'd1,sample_bins[9:5]};state<=READ_BIN;end
                SNAP_READ: state<=SNAP_WRITE;
                SNAP_WRITE: if(addr>=96) begin
                        // Exact floor(95 * bin_count / sampled_pixels), once
                        // per frame in blanking. Serial division, no DSP.
                        y_numerator<=({7'd0,count_q}<<6)+({7'd0,count_q}<<5)-count_q;
                        y_remainder<=0;y_quotient<=0;y_step<=22;state<=Y_SCALE;
                    end else begin addr<=addr+1'b1;state<=SNAP_READ;end
                Y_SCALE: begin
                    y_numerator<=y_numerator<<1;
                    y_remainder<=y_subtract ? y_trial-SAMPLE_COUNT : y_trial;
                    y_quotient<={y_quotient[5:0],y_subtract};
                    if(y_step==0) state<=Y_COMMIT;else y_step<=y_step-1'b1;
                end
                Y_COMMIT: begin
                    if(addr==96) begin
                        y_previous<=y_quotient;addr<=addr+1'b1;state<=SNAP_READ;
                    end else begin
                        // Compute the exact old interpolation in blanking,
                        // rather than ERAM -> subtract -> three adds -> min/
                        // max -> compare on every displayed pixel. A running
                        // numerator advances by one slope per output column.
                        y_slope<=$signed({4'd0,y_quotient})-$signed({4'd0,y_previous});
                        y_position<=$signed({1'b0,y_previous,3'd0});
                        y_fraction<=0;state<=Y_NEXT;
                    end
                end
                Y_NEXT: begin y_next_position<=y_position+y_slope;state<=Y_PIXEL;end
                Y_PIXEL: begin
                    y_position<=y_next_position;
                    if(y_fraction==7) begin
                        y_previous<=y_quotient;y_fraction<=0;
                        if(addr==127) state<=Y_LAST;
                        else begin addr<=addr+1'b1;state<=SNAP_READ;end
                    end else begin y_fraction<=y_fraction+1'b1;state<=Y_NEXT;end
                end
                Y_LAST: if(y_fraction==7) state<=Y_PUBLISH;
                    else y_fraction<=y_fraction+1'b1;
                // Publish after the final RAM write, allowing its synchronous
                // read to settle. A partially refreshed table is never drawn.
                Y_PUBLISH: begin valid<=1;addr<=0;state<=IDLE;end
                default: state<=CLEAR;
            endcase
        end
    end
    wire [11:0] dx=draw_x-PANEL_X,dy=draw_y-PANEL_Y;
    wire panel=draw_de && enable && draw_x>=PANEL_X && draw_x<PANEL_X+256 &&
               draw_y>=PANEL_Y && draw_y<PANEL_Y+224 && dy[6:0]<96;
    // One shared RGB plot: 2-pixel R/G/B bars plus a 2-pixel gap per bin.
    // The second plot is Y. One synchronous read port suffices for both.
    wire [1:0] draw_channel=dy[7] ? 2'd3 : dx[2:1];
    wire draw_bar=dy[7] || dx[2:1]!=2'd3;
    reg [13:0] height_q;
    reg [6:0] row_q;
    reg [1:0] ch_q;
    reg panel_q,grid_q,bar_q,y_plot_q;
    // Row bounds already include the connection to the next column. The
    // one-row antialias margin is identical to the previous renderer.
    wire y_line=({1'b0,row_q}+8'd1>={1'b0,height_q[6:0]}) &&
                ({1'b0,row_q}<={1'b0,height_q[13:7]}+8'd1);
    // Two cycles match the mixer's existing DE_2d; no added video latency.
    wire [8:0] draw_addr=dy[7] ? {1'b1,dx[7:0]} : {2'd0,draw_channel,dx[7:3]};
    always @(posedge clk) height_q<=heights[draw_addr];
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin panel_q<=0;grid_q<=0;bar_q<=0;y_plot_q<=0;row_q<=0;ch_q<=0;hit<=0;colour<=0;end
        else begin
            panel_q<=panel; row_q<=95-dy[6:0];ch_q<=draw_channel;bar_q<=draw_bar;
            y_plot_q<=dy[7];
            grid_q<=dx[7:0]==0 || dx[7:0]==255 || dy[4:0]==31;
            hit<=panel_q;
            colour<=grid_q ? 24'h344256 : 24'h101722;
            if(valid && y_plot_q && y_line) colour<=24'he8edf4;
            else if(valid && ch_q!=3 && bar_q && height_q[6:0]!=0 && row_q<height_q[6:0]) begin
                case(ch_q)
                    0:colour<=24'he85c62;
                    1:colour<=24'h54ca89;
                    2:colour<=24'h579ced;
                    default:colour<=24'h101722;
                endcase
            end
        end
    end
endmodule

// Gain registers published only after all sensor writes ACKed. Output is x1000.
module camera_gain_values(input wire clk,input wire [15:0] again,dgain,
                          output reg [15:0] again_milli,dgain_milli);
    reg [9:0] aq,dq;
    wire [1:0] ac=again[12] ? 3 : again[11] ? 2 : again[10] ? 1 : 0;
    wire [1:0] dc=dgain[10] ? 3 : dgain[9] ? 2 : dgain[8] ? 1 : 0;
    always @(posedge clk) begin
        aq<={4'd0,again[5:0]}<<ac;
        dq<={4'd0,dgain[7:2]}<<dc;
        // q/32 * 1000 = q*125/4. Shifts/adds, no divider or multiplier.
        again_milli<=(({7'd0,aq}<<7)-({7'd0,aq}<<1)-aq+17'd2)>>2;
        dgain_milli<=(({7'd0,dq}<<7)-({7'd0,dq}<<1)-dq+17'd2)>>2;
    end
endmodule

// Read-only display levels: exposure 64..1990, gains table index 0..127.
// One shared restoring divider, 20 cycles per value. Snapshot matches the BCD
// formatter's frame start; all three lengths publish together before row 464.
module camera_parameter_levels(
    input wire clk,rst_n,start,
    input wire [15:0] exposure,again,dgain,
    output reg [26:0] levels
);
    wire [1:0] ac=again[12] ? 3 : again[11] ? 2 : again[10] ? 1 : 0;
    wire [1:0] dc=dgain[10] ? 3 : dgain[9] ? 2 : dgain[8] ? 1 : 0;
    reg [10:0] exp_offset;
    reg [6:0] ag_index,dg_index;
    reg [1:0] state,channel;
    reg [4:0] step;
    reg [19:0] numerator;
    reg [10:0] denominator,remainder;
    reg [8:0] quotient;
    reg [17:0] work_levels;
    wire [10:0] value=channel==0 ? exp_offset :
                             channel==1 ? {4'd0,ag_index} : {4'd0,dg_index};
    wire [11:0] trial={remainder,numerator[19]};
    wire subtract=trial>={1'b0,denominator};
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            levels<=0;exp_offset<=0;ag_index<=0;dg_index<=0;
            state<=0;channel<=0;step<=0;numerator<=0;denominator<=1;
            remainder<=0;quotient<=0;work_levels<=0;
        end else case(state)
            0: if(start) begin
                exp_offset<=exposure<64 ? 11'd0 : exposure>1990 ? 11'd1926 : exposure-16'd64;
                ag_index<={ac,again[4:0]};dg_index<={dc,dgain[6:2]};
                channel<=0;state<=1;
            end
            1: begin
                numerator<=({9'd0,value}<<8)+({9'd0,value}<<6);
                denominator<=channel==0 ? 11'd1926 : 11'd127;
                remainder<=0;quotient<=0;step<=19;state<=2;
            end
            2: begin
                numerator<=numerator<<1;
                remainder<=subtract ? trial-{1'b0,denominator} : trial;
                quotient<={quotient[7:0],subtract};
                if(step==0) state<=3;else step<=step-1'b1;
            end
            3: begin
                if(channel==0) work_levels[8:0]<=quotient;
                if(channel==1) work_levels[17:9]<=quotient;
                if(channel==2) begin levels<={quotient,work_levels};state<=0;end
                else begin channel<=channel+1'b1;state<=1;end
            end
        endcase
    end
endmodule
