/*****************************************************************
Company : MiLianKe Electronic Technology Co., Ltd.
WebSite:https://www.milianke.com
TechWeb:https://www.uisrc.com
tmall-shop:https://milianke.tmall.com
jd-shop:https://milianke.jd.com
taobao-shop: https://milianke.taobao.com
Description: 
The reference demo provided by Milianke is only used for learning. 
We cannot ensure that the demo itself is free of bugs, so users 
should be responsible for the technical problems and consequences
caused by the use of their own products.
@Author      :   XiaoQingquan 
@Time        :   2024/10 
version:     :   
@Description :   在原基础上删去了一些赘余的代码，并且优化了时序，加入BRAM IP核，
                 让从除法器出来的每一个数据都可以对上相应的像素数据进行增益，使
                 得它的适用行更加广泛了。
*****************************************************************/
module awb  #(   
    parameter IMG_HEIGHT = 1080,
    parameter IMG_WIDTH  = 1920,
    parameter integer MIN_STATS_PIXELS = 0,
    parameter integer GAIN_STEP_Q16 = 4096
) 
(
    input                   I_clk  ,
    input                   I_rst_n,
    input                   I_enable,


    input                   I_tlast  ,
    input                   I_tuser  ,
    input [95:0]            I_tdata  ,
    input                   I_tvalid , 
    output                  I_tready ,

    output                  O_tlast  ,
    output                  O_tuser  ,
    output [95:0]           O_tdata  ,
    output                  O_tvalid ,
    input                   O_tready
);

/*****************************************************************
                          RGB565תRGB888                                        
*****************************************************************/

    // //RGB565??????888
    // wire [4:0]  rgb565_r   = I_st_data[11+:5];
    // wire [5:0]  rgb565_g   = I_st_data[5+:6];
    // wire [4:0]  rgb565_b   = I_st_data[0+:5];
    
    // wire [23:0] rgb888     = {{rgb565_r[4:0],rgb565_r[2:0]},{rgb565_g[5:0],rgb565_g[1:0]},{rgb565_b[4:0],rgb565_b[2:0]}};
    
    // wire [7:0]  rgb888_r   = rgb888[16+:8];
    // wire [7:0]  rgb888_g   = rgb888[8+:8] ;
    // wire [7:0]  rgb888_b   = rgb888[0+:8] ;
    
  /*****************************************************************
                          中间信号                                        
*****************************************************************/


  reg         I_tvalid_r0,I_tvalid_r1;
  reg  [95:0] I_tdata_d0,I_tdata_d1;


  //累加RGB通道的�??
  reg  [31:0]         sum_r[0:3];
  reg  [31:0]         sum_g[0:3];
  reg  [31:0]         sum_b[0:3];

  
  wire  [95:0]        data_d0 ;
  wire                valid_d0;
  wire                I_tuser_r0;    

  reg                 I_tuser_r1;

  reg   [95:0]        data_d1 ;
  reg                 valid_d1;
  reg   [9:0]         rgb_sum[0:3];//每个像素点RGB的和
  /*****************************************************************
                            平均值计�??                                      
*****************************************************************/

// 对输入数据和有效信号进行寄存，形成流水线
always @(posedge I_clk or negedge I_rst_n) begin
    if (!I_rst_n) begin
        I_tdata_d0  <= 0;
        I_tdata_d1  <= 0;
        {I_tvalid_r0,I_tvalid_r1} <= 0;
    end else begin
        I_tdata_d0  <= I_tdata   ;
        I_tdata_d1  <= I_tdata_d0;
        I_tvalid_r0 <= I_tvalid;
        I_tvalid_r1 <= I_tvalid_r0;
    end
end

// Confidence-gated grey-world statistics over all four pixel lanes.
// Reject dark, clipped and strongly coloured samples so a tracked colour
// target cannot drive the illuminant estimate toward its own complement.
localparam integer AUTO_MIN_STATS=(IMG_WIDTH*IMG_HEIGHT)/256;
localparam integer MIN_SAMPLES=(MIN_STATS_PIXELS>0) ? MIN_STATS_PIXELS :
                              ((AUTO_MIN_STATS<16) ? 16 : AUTO_MIN_STATS);
reg [3:0] neutral_pixel;
wire [7:0] stats_r[0:3],stats_g[0:3],stats_b[0:3];
reg [1:0] stats_input_frame;
// The image already has two input registers. Reuse I_tdata_d1/I_tvalid_r1
// for the matching raw sample, and put two statistic-only register boundaries
// before the pair adders. Pixel latency and the neutral-sample rule are intact.
always @(posedge I_clk or negedge I_rst_n) begin
    if(!I_rst_n) stats_input_frame<=0;
    else stats_input_frame<={stats_input_frame[0],I_tuser};
end
genvar stats_lane;
generate for(stats_lane=0;stats_lane<4;stats_lane=stats_lane+1) begin: g_stats_pixel
    wire [7:0] r=I_tdata[stats_lane*24+16+:8];
    wire [7:0] g=I_tdata[stats_lane*24+8+:8];
    wire [7:0] b=I_tdata[stats_lane*24+:8];
    wire [7:0] rg_low=(r<g) ? r : g;
    wire [7:0] rg_high=(r>g) ? r : g;
    reg [7:0] low_q,high_q;
    wire [9:0] neutral_limit={2'b0,low_q}+{1'b0,low_q,1'b0};
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            low_q<=0;high_q<=0;neutral_pixel[stats_lane]<=0;
        end else begin
            low_q<=(b<rg_low) ? b : rg_low;
            high_q<=(b>rg_high) ? b : rg_high;
            neutral_pixel[stats_lane]<=low_q>=12 && high_q<=240 &&
                {2'b0,high_q}<=neutral_limit;
        end
    end
    assign stats_r[stats_lane]=neutral_pixel[stats_lane] ?
        I_tdata_d1[stats_lane*24+16+:8] : 8'd0;
    assign stats_g[stats_lane]=neutral_pixel[stats_lane] ?
        I_tdata_d1[stats_lane*24+8+:8] : 8'd0;
    assign stats_b[stats_lane]=neutral_pixel[stats_lane] ?
        I_tdata_d1[stats_lane*24+:8] : 8'd0;
end endgenerate
reg [8:0] pair_r0,pair_r1,pair_g0,pair_g1,pair_b0,pair_b1;
reg [1:0] pair_n0,pair_n1;
reg [9:0] beat_r,beat_g,beat_b;
reg [2:0] beat_count;
reg stats_valid1,stats_valid2,stats_frame1,stats_frame2;
reg [31:0] stats_count;
reg frame_enable;
// Balanced, registered reduction keeps the extra sample selection out of the
// existing pixel multipliers and avoids a four-adder accumulation path.
always @(posedge I_clk or negedge I_rst_n) begin
    if(!I_rst_n) begin
        pair_r0<=0;pair_r1<=0;pair_g0<=0;pair_g1<=0;pair_b0<=0;pair_b1<=0;
        pair_n0<=0;pair_n1<=0;beat_r<=0;beat_g<=0;beat_b<=0;beat_count<=0;
        stats_valid1<=0;stats_valid2<=0;stats_frame1<=0;stats_frame2<=0;
        sum_r[0]<=0;sum_g[0]<=0;sum_b[0]<=0;stats_count<=0;frame_enable<=0;
    end else begin
        if(I_tuser) frame_enable<=I_enable;
        pair_r0<={1'b0,stats_r[0]}+{1'b0,stats_r[1]};
        pair_r1<={1'b0,stats_r[2]}+{1'b0,stats_r[3]};
        pair_g0<={1'b0,stats_g[0]}+{1'b0,stats_g[1]};
        pair_g1<={1'b0,stats_g[2]}+{1'b0,stats_g[3]};
        pair_b0<={1'b0,stats_b[0]}+{1'b0,stats_b[1]};
        pair_b1<={1'b0,stats_b[2]}+{1'b0,stats_b[3]};
        pair_n0<={1'b0,neutral_pixel[0]}+{1'b0,neutral_pixel[1]};
        pair_n1<={1'b0,neutral_pixel[2]}+{1'b0,neutral_pixel[3]};
        beat_r<={1'b0,pair_r0}+{1'b0,pair_r1};
        beat_g<={1'b0,pair_g0}+{1'b0,pair_g1};
        beat_b<={1'b0,pair_b0}+{1'b0,pair_b1};
        beat_count<={1'b0,pair_n0}+{1'b0,pair_n1};
        stats_valid1<=I_tvalid_r1;stats_valid2<=stats_valid1;
        stats_frame1<=stats_input_frame[1];stats_frame2<=stats_frame1;
        if(stats_frame2) begin
            // If SOF accompanies the first valid beat, include that beat.
            sum_r[0]<=stats_valid2 ? {22'd0,beat_r} : 32'd0;
            sum_g[0]<=stats_valid2 ? {22'd0,beat_g} : 32'd0;
            sum_b[0]<=stats_valid2 ? {22'd0,beat_b} : 32'd0;
            stats_count<=stats_valid2 ? {29'd0,beat_count} : 32'd0;
        end else if(stats_valid2) begin
            sum_r[0]<=sum_r[0]+beat_r;
            sum_g[0]<=sum_g[0]+beat_g;
            sum_b[0]<=sum_b[0]+beat_b;
            stats_count<=stats_count+beat_count;
        end
    end
end
wire stats_confident=(stats_count>=MIN_SAMPLES);

    // A single iterative divider calculates two ratios once per frame.
    // The pixel path and its line-buffer timing remain unchanged.
    wire [19:0] in_r_r1, in_b_r1;
    awb_frame_gain #(
        .MIN_SUM(MIN_SAMPLES*12-1),.MIN_GAIN(32768),.MAX_GAIN(196608),
        .STEP_Q16(GAIN_STEP_Q16)
    ) u_frame_gain (
        .I_clk(I_clk), .I_rst_n(I_rst_n), .I_frame_start(stats_frame2),
        .I_sum_r(stats_confident ? sum_r[0] : 32'd0),
        .I_sum_g(stats_confident ? sum_g[0] : 32'd0),
        .I_sum_b(stats_confident ? sum_b[0] : 32'd0),
        .O_gain_r(in_r_r1), .O_gain_b(in_b_r1)
    );

     
    signal_delay #(
        .IMG_HEIGHT   (IMG_HEIGHT),
        .IMG_WIDTH    (IMG_WIDTH ),
        .DATA_WIDTH   (96  ),
        .DELAY_CYCLE  (50)
    )signal_delay_d (
        /*input                       */ .I_clk  (I_clk  ),
        /*input                       */ .I_rst_n(I_rst_n),
        /*input                       */ .I_tuser(I_tuser),  //synthesis keep 
        /*input  wire                 */ .I_valid(I_tvalid_r1),  //synthesis keep 
        /*input  wire [DATA_WIDTH-1:0]*/ .I_data (I_tdata_d1 ),   //synthesis keep 
        /*output wire                 */ .O_valid(valid_d0   ),  //synthesis keep 
        /*output wire [DATA_WIDTH-1:0]*/ .O_data (data_d0    ),   //synthesis keep 
        /*output wire                 */ .O_tuser(I_tuser_r0 )
    );

    

/*****************************************************************
                            ��������                                      
*****************************************************************/



    ////信号同步
    always @(posedge I_clk or negedge I_rst_n) begin
      if (!I_rst_n) begin
          valid_d1 <= 0;
          data_d1  <= 0;
          I_tuser_r1 <= 0;
      end else begin
          valid_d1 <= valid_d0;
          data_d1  <= data_d0 ;
          I_tuser_r1 <= I_tuser_r0;
      end
    end
/*****************************************************************
                           ��������                                       
*****************************************************************/

    reg [27:0] R_new_r1;
    reg [27:0] G_new_r1;
    reg [27:0] B_new_r1;

    reg [27:0] R_new_r2;
    reg [27:0] G_new_r2;
    reg [27:0] B_new_r2;

    reg [27:0] R_new_r3;
    reg [27:0] G_new_r3;
    reg [27:0] B_new_r3;

    reg [27:0] R_new_r4;
    reg [27:0] G_new_r4;
    reg [27:0] B_new_r4;


    wire   [7:0] rgb888_r_d1[0:3];
    wire   [7:0] rgb888_g_d1[0:3];
    wire   [7:0] rgb888_b_d1[0:3];

    assign  rgb888_r_d1[0] = data_d1[16+:8];
    assign  rgb888_g_d1[0] = data_d1[8+:8];
    assign  rgb888_b_d1[0] = data_d1[0+:8];

    assign  rgb888_r_d1[1] = data_d1[40+:8];
    assign  rgb888_g_d1[1] = data_d1[32+:8];
    assign  rgb888_b_d1[1] = data_d1[24+:8];

    assign  rgb888_r_d1[2] = data_d1[64+:8];
    assign  rgb888_g_d1[2] = data_d1[56+:8];
    assign  rgb888_b_d1[2] = data_d1[48+:8];

    assign  rgb888_r_d1[3] = data_d1[88+:8];
    assign  rgb888_g_d1[3] = data_d1[80+:8];
    assign  rgb888_b_d1[3] = data_d1[72+:8];
    always @(posedge I_clk or negedge I_rst_n)begin 
        if(!I_rst_n)begin
            R_new_r1 <= 0;
            G_new_r1 <= 0;
            B_new_r1 <= 0;

            R_new_r2 <= 0;
            G_new_r2 <= 0;
            B_new_r2 <= 0;

            R_new_r3 <= 0;
            G_new_r3 <= 0;
            B_new_r3 <= 0;

            R_new_r4 <= 0;
            G_new_r4 <= 0;
            B_new_r4 <= 0;
        end
        else if(valid_d1) begin
            R_new_r1 <= in_r_r1 * rgb888_r_d1[0];
            G_new_r1 <=           rgb888_g_d1[0];
            B_new_r1 <= in_b_r1 * rgb888_b_d1[0];

            R_new_r2 <= in_r_r1 * rgb888_r_d1[1];
            G_new_r2 <=           rgb888_g_d1[1];
            B_new_r2 <= in_b_r1 * rgb888_b_d1[1];

            R_new_r3 <= in_r_r1 * rgb888_r_d1[2];
            G_new_r3 <=           rgb888_g_d1[2];
            B_new_r3 <= in_b_r1 * rgb888_b_d1[2];

            R_new_r4 <= in_r_r1 * rgb888_r_d1[3];
            G_new_r4 <=           rgb888_g_d1[3];
            B_new_r4 <= in_b_r1 * rgb888_b_d1[3];
        end
    end

/*****************************************************************
                            �������???                                      
*****************************************************************/

    wire [7:0] R_new_x1;
    wire [7:0] G_new_x1;
    wire [7:0] B_new_x1;

    wire [7:0] R_new_x2;
    wire [7:0] G_new_x2;
    wire [7:0] B_new_x2;

    wire [7:0] R_new_x3;
    wire [7:0] G_new_x3;
    wire [7:0] B_new_x3;

    wire [7:0] R_new_x4;
    wire [7:0] G_new_x4;
    wire [7:0] B_new_x4;

    assign R_new_x1 = (R_new_r1[27:24])?255:R_new_r1[23:16];
    assign G_new_x1 =  G_new_r1;
    assign B_new_x1 = (B_new_r1[27:24])?255:B_new_r1[23:16];

    assign R_new_x2 = (R_new_r2[27:24])?255:R_new_r2[23:16];
    assign G_new_x2 =  G_new_r2;
    assign B_new_x2 = (B_new_r2[27:24])?255:B_new_r2[23:16];

    assign R_new_x3 = (R_new_r3[27:24])?255:R_new_r3[23:16];
    assign G_new_x3 =  G_new_r3;
    assign B_new_x3 = (B_new_r3[27:24])?255:B_new_r3[23:16];

    assign R_new_x4 = (R_new_r4[27:24])?255:R_new_r4[23:16];
    assign G_new_x4 =  G_new_r4;
    assign B_new_x4 = (B_new_r4[27:24])?255:B_new_r4[23:16];
/*****************************************************************
                           ͬ���ź�                                       
*****************************************************************/

    reg valid_d2,I_tuser_r2;
    reg [95:0] raw_data_d2;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)begin
            valid_d2 <= 0;
            I_tuser_r2 <= 0;
            raw_data_d2 <= 0;
        end
        else begin
            valid_d2 <= valid_d1;
            I_tuser_r2 <= I_tuser_r1;
            raw_data_d2 <= data_d1;
        end
    end


    assign  O_tlast  = (valid_d1 == 0) && (valid_d2 == 1);   
    // Keep the original frame-start relationship used by this line-buffered
    // block; downstream framing was already verified with this connection.
    assign  O_tuser  = I_tuser;
    assign  O_tvalid = valid_d2;

    assign  O_tdata = O_tvalid ?
                      (frame_enable ? {R_new_x4,G_new_x4,B_new_x4,
                                   R_new_x3,G_new_x3,B_new_x3,
                                   R_new_x2,G_new_x2,B_new_x2,
                                   R_new_x1,G_new_x1,B_new_x1}
                                : raw_data_d2)
                      : 96'd0;
    // assign O_t_data = {R_new[7-:5],G_new[7-:6],B_new[7-:5]};
    assign  I_tready = O_tready;

endmodule

// Frame-rate Q16 grey-world gain estimator: one 48/32 restoring divider,
// reused for red and blue (96 clocks total). Never a per-pixel divider.
module awb_frame_gain #(
    parameter integer MIN_SUM=131072,
    parameter integer MIN_GAIN=16384,
    parameter integer MAX_GAIN=262144,
    parameter integer STEP_Q16=0
) (
    input wire I_clk,
    input wire I_rst_n,
    input wire I_frame_start,
    input wire [31:0] I_sum_r, I_sum_g, I_sum_b,
    output reg [19:0] O_gain_r, O_gain_b
);
    reg [1:0] state;
    reg [5:0] bit_count;
    reg [47:0] dividend, quotient;
    reg [31:0] denominator, remainder;
    reg [31:0] sum_g_hold, sum_b_hold;
    reg [19:0] pending_r, pending_b;
    reg pending_valid;
    wire [32:0] trial = {remainder,dividend[47]};
    wire subtract = trial >= {1'b0,denominator};
    wire [32:0] residue = subtract ? trial-{1'b0,denominator} : trial;
    wire [47:0] next_quotient = {quotient[46:0],subtract};
    function [19:0] clamp_gain;
        input [47:0] q;
        begin
            if(q > MAX_GAIN) clamp_gain=MAX_GAIN;
            else if(q < MIN_GAIN) clamp_gain=MIN_GAIN;
            else clamp_gain=q[19:0];
        end
    endfunction
    function [19:0] slewed_gain;
        input [19:0] current,target;
        begin
            if(STEP_Q16==0) slewed_gain=target;
            else if(target>current && target-current>STEP_Q16)
                slewed_gain=current+STEP_Q16;
            else if(current>target && current-target>STEP_Q16)
                slewed_gain=current-STEP_Q16;
            else slewed_gain=target;
        end
    endfunction
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            state<=0; bit_count<=0; dividend<=0; quotient<=0;
            denominator<=1; remainder<=0; sum_g_hold<=0; sum_b_hold<=1;
            pending_r<=20'h10000; pending_b<=20'h10000; pending_valid<=0;
            O_gain_r<=20'h10000; O_gain_b<=20'h10000;
        end else begin
            // Commit both completed ratios together, only at frame boundaries.
            if(I_frame_start && pending_valid) begin
                O_gain_r<=slewed_gain(O_gain_r,pending_r); O_gain_b<=slewed_gain(O_gain_b,pending_b); pending_valid<=0;
            end
            if(state==0) begin
                if(I_frame_start && I_sum_r>MIN_SUM &&
                   I_sum_g>MIN_SUM && I_sum_b>MIN_SUM) begin
                    sum_g_hold<=I_sum_g; sum_b_hold<=I_sum_b;
                    dividend<={I_sum_g,16'd0}; denominator<=I_sum_r;
                    quotient<=0; remainder<=0; bit_count<=47; state<=1;
                end
            end else begin
                dividend<={dividend[46:0],1'b0};
                quotient<=next_quotient;
                remainder<=residue[31:0];
                bit_count<=bit_count-1'b1;
                if(bit_count==0) begin
                    if(state==1) begin
                        pending_r<=clamp_gain(next_quotient);
                        dividend<={sum_g_hold,16'd0}; denominator<=sum_b_hold;
                        quotient<=0; remainder<=0; bit_count<=47; state<=2;
                    end else begin
                        pending_b<=clamp_gain(next_quotient);
                        pending_valid<=1; state<=0;
                    end
                end
            end
        end
    end
endmodule



