module isp_top #(
    parameter integer IMG_WIDTH = 1024,
    parameter integer IMG_HEIGHT = 600,
    parameter integer DETECT_SUPPORT = 0
) (
    input 			axi4s_video_aclk,
    input 			I_rst_n			,
    input [2:0] 	I_mode			,   // 图像处理模式（已同步到本时钟域）
    input           I_awb_enable    ,   // 1=AWB，0=保持 demosaic 原始 RGB
    input           I_detect_switch_n,
    output          O_detect_result_strobe,
    output [438:0]  O_detect_result,
    output [15:0]   O_detect_capture_id,
    output          O_detect_bad,
    input 			I_tlast			,
    input 			I_tuser			,
    input [39:0] 	I_tdata			,
    input 			I_tvalid		,
    input [9:0] 	I_tdest			,
    input 			O_tready		,
    output [127:0] 	O_tdata			,
    output 			O_tlast			,
    output 			O_tuser			,
    output 			O_tvalid		,
    output 			I_tready        ,

    // Native four-pixel RGB888 stream for the Ethernet side path. Keeping
    // this tap before 96->128 repacking avoids throwing away any pixels.
    output [95:0]    O_eth_tdata     ,
    output           O_eth_tuser     ,
    output           O_eth_tvalid    ,

    // 中央 ROI 的去黑电平、demosaic 后线性 RGB，AWB 前；CCM 当前为恒等。
    output [7:0]     O_cal_r_avg     ,
    output [7:0]     O_cal_g_avg     ,
    output [7:0]     O_cal_b_avg     ,
    output [9:0]     O_cal_clip_permille,
    output           O_cal_update_toggle
);
 
wire         m_aixs_tvalid;  //synthesis keep 
  wire [127:0] m_aixs_tdata;  //synthesis keep 
  wire         m_aixs_tuser;  //synthesis keep 
  wire         m_aixs_tlast;  //synthesis keep 
  wire         m_aixs_tready;  //synthesis keep 
  
  wire [ 95:0] m_aixs_tdata_96;  //synthesis keep 

  wire [ 39:0] blc_tdata;       //synthesis keep
  wire         ccm_O_tlast;     //synthesis keep
  wire         ccm_O_tuser;     //synthesis keep
  wire [ 95:0] ccm_O_tdata;     //synthesis keep
  wire         ccm_O_tvalid;    //synthesis keep


  wire         awb_O_tlast;  //synthesis keep 
  wire         awb_O_tuser;  //synthesis keep 
  wire [ 95:0] awb_O_tdata;  //synthesis keep 
  wire         awb_O_tvalid;  //synthesis keep 
  wire         awb_O_tready;  //synthesis keep 

  wire         awb_O_tuser_128;  //synthesis keep
  wire         awb_O_tvalid_128;  //synthesis keep
  wire [127:0] awb_O_tdata_128;  //synthesis keep

    wire         edge_O_tlast;  //synthesis keep
    wire         edge_O_tuser;  //synthesis keep
    wire [ 95:0] edge_O_tdata;  //synthesis keep
    wire         edge_O_tvalid;  //synthesis keep
    wire         edge_O_tready;  //synthesis keep

    // video_proc's legacy ready ports have reversed names; connect exactly once.
    assign awb_O_tready = O_tready;
  assign O_tdata  = awb_O_tdata_128;
  // 输出控制信号必须和 video_proc_96 处理后的像素处于同一流水级。
  // 原先这里误用了 AWB 的行尾信号，比 O_tdata/O_tvalid 提前 9 拍。
  assign O_tlast  = edge_O_tlast   ;
  assign O_tuser  = awb_O_tuser_128    ;
  assign O_tvalid = awb_O_tvalid_128   ;
  assign O_eth_tdata  = edge_O_tdata;
  assign O_eth_tuser  = edge_O_tuser;
  assign O_eth_tvalid = edge_O_tvalid;
  
  //assign m_aixs_tready = O_tready;
  //assign O_tdata  = m_aixs_tdata;
  //assign O_tlast  = m_aixs_tlast    ;
  //assign O_tuser  = m_aixs_tuser    ;
  //assign O_tvalid = m_aixs_tvalid   ;
 
// 遮光实测四个 Bayer 通道黑电平均为 64。RAW 统计仍在本模块外观察
// 未校正值；图像路径扣除 pedestal 后，将 [64,1023] 归一化到 [0,1023]。
// 不能只减 64 再直接取高 8 位，否则满白会被压成 239。
// Register RAW and its framing together before normalization. Without this
// boundary, crop-counter decode + subtract + scale ran through one 8.888ns path.
reg [39:0] raw_input_data;
reg raw_input_valid, raw_input_last, raw_input_user;
always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
    if(!I_rst_n) begin
        raw_input_data <= 40'd0;
        raw_input_valid <= 1'b0;
        raw_input_last <= 1'b0;
        raw_input_user <= 1'b0;
    end else begin
        raw_input_data <= I_tdata;
        raw_input_valid <= I_tvalid;
        raw_input_last <= I_tlast;
        raw_input_user <= I_tuser;
    end
end
raw_black_level_corrector #(
    .BLACK_LEVEL(10'd64)
) u_raw_black_level_corrector (
    .I_tdata(raw_input_data),
    .O_tdata(blc_tdata)
);

demosaic #(
    .IMG_HEIGHT          (IMG_HEIGHT),
    .IMG_WIDTH           (IMG_WIDTH),
    .data_complete_delay (50  ),
    .BAYER_MODE          ("BGGR")
)
u_demosaic
(
    .I_clk   			(axi4s_video_aclk)	,   // 时钟信号
    .I_rst_n 			(I_rst_n)	,   // 复位信号，低有效
    .axi4s_video_tdata 	(blc_tdata)	,  // 黑电平校正后的 RAW10
    .axi4s_video_tdest 	(I_tdest)	,
    .axi4s_video_tlast 	(raw_input_last)	,  // 与 RAW 同拍寄存
    .axi4s_video_tvalid	(raw_input_valid)	,
    .axi4s_video_tuser 	(raw_input_user)	,
    .axi4s_video_tready	(I_tready)	,  // 从模块准备好接受数据
    .O_tlast  			(m_aixs_tlast)	,  // 输出行结束信号
    .O_tuser  			(m_aixs_tuser)	,  // 输出帧开始信号
    .O_tdata  			(m_aixs_tdata)	,  // 输出数据
    .O_tvalid 			(m_aixs_tvalid)	,  // 输出数据有效信号
    .O_tready   		(m_aixs_tready)	   // 输出数据准备好信号
);
 
data128_96 u_data128_96 (
    .I_tdata(m_aixs_tdata),
    .O_tdata(m_aixs_tdata_96)
);

// 关闭未经有效色卡标定的矩阵和硬阈值绿色补偿，只保留等长流水延迟。
// SW0 仍控制后面的 AWB；关闭时观察传感器线性 RGB，而非标准 sRGB。
color_correction_matrix #(.ENABLE(0)) u_color_correction_matrix (
    .I_clk   (axi4s_video_aclk),
    .I_rst_n (I_rst_n),
    .I_tlast (m_aixs_tlast),
    .I_tuser (m_aixs_tuser),
    .I_tdata (m_aixs_tdata_96),
    .I_tvalid(m_aixs_tvalid),
    .O_tlast (ccm_O_tlast),
    .O_tuser (ccm_O_tuser),
    .O_tdata (ccm_O_tdata),
    .O_tvalid(ccm_O_tvalid)
);

calibration_rgb_stats #(.IMG_WIDTH(IMG_WIDTH), .IMG_HEIGHT(IMG_HEIGHT)) u_calibration_rgb_stats (
    .I_clk            (axi4s_video_aclk),
    .I_rst_n          (I_rst_n),
    .I_tuser          (ccm_O_tuser),
    .I_tlast          (ccm_O_tlast),
    .I_tvalid         (ccm_O_tvalid),
    .I_tdata          (ccm_O_tdata),
    .O_r_avg          (O_cal_r_avg),
    .O_g_avg          (O_cal_g_avg),
    .O_b_avg          (O_cal_b_avg),
    .O_clip_permille  (O_cal_clip_permille),
    .O_update_toggle  (O_cal_update_toggle)
);

// 保留原工程已在板上验证的 CCM -> AWB 时序关系。AWB 内部带
// 行缓存，不能当作普通组合/短流水模块任意前移。
awb #(
    .IMG_HEIGHT(IMG_HEIGHT),
    .IMG_WIDTH (IMG_WIDTH)
) u_awb (
    .I_clk   (axi4s_video_aclk),
    .I_rst_n (I_rst_n),
    .I_enable(I_awb_enable),
    .I_tlast (ccm_O_tlast),
    .I_tuser (ccm_O_tuser),
    .I_tdata (ccm_O_tdata),
    .I_tvalid(ccm_O_tvalid),
    .I_tready(m_aixs_tready),
    .O_tlast (awb_O_tlast ),
    .O_tuser (awb_O_tuser ),
    .O_tdata (awb_O_tdata ),
    .O_tvalid(awb_O_tvalid),
    .O_tready(awb_O_tready)
);

wire [2:0] effective_mode;
generate if(DETECT_SUPPORT) begin : g_detect
    wire detect_enabled;
    wire [15:0] frame_id;
    wire [422:0] detected_boxes;
    detect_control u_control(
        .clk(axi4s_video_aclk),.rst_n(I_rst_n),.switch_n(I_detect_switch_n),
        .sof(awb_O_tuser),.selected_mode(I_mode),.enabled(detect_enabled),.mode(effective_mode));
    color_shape_detector #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) u_detector(
        .clk(axi4s_video_aclk),.rst_n(I_rst_n),.enable(detect_enabled),
        .sof(edge_O_tuser),.valid(edge_O_tvalid),.rgb(edge_O_tdata),
        .result_strobe(O_detect_result_strobe),.result_id(frame_id),
        .boxes(detected_boxes),.busy(),.overflow(),
        .capture_frame_id(O_detect_capture_id));
    // Label exhaustion is a capacity warning, not a corrupt-frame indication.
    // Each empty-row-separated band filters incomplete components on overflow;
    // remaining complete boxes must still be eligible for tracking.
    assign O_detect_bad=1'b0;
    assign O_detect_result={frame_id,detected_boxes};
end else begin : g_no_detect
    assign effective_mode=I_mode;
    assign O_detect_result_strobe=1'b0;
    assign O_detect_result=439'd0;
    assign O_detect_capture_id=16'd0;
    assign O_detect_bad=1'b0;
end endgenerate

// Display gamma follows full-range RAW normalization and frame-based AWB.
// All three channels use the same gamma-2.2 curve with white point 255, so
// neutral AWB output remains neutral and normalized highlights are preserved.
// HDMI and UDP share this curve before the display modes and colour detector.
// CURVE_SEL: 0=1.8 1=2.0 2=2.2 3=2.4 4=linear 5=legacy chart RGB curves.
// Preset 5 remains available for comparison; its captured chart fit is unchanged.
localparam integer CURVE_SEL = 2;
// Do not instantiate the measured 3x3 matrix in the production stream.  Its
// four-pixel implementation asks TD for 36 DSPs; the whole design then needs
// 61 on a 40-DSP device.  Routing skipped multiplier nets and corrupted RGB on
// board.
// Instead, restore sensor colour saturation with a DSP-free uniform matrix
// c' = Y + k*(c-Y), Y=(R+2G+B)/4, in linear light after AWB.  Fitting k alone
// to the 2026-09-14 chart: chroma RMSE 0.242 identity, 0.135 at k=1.875,
// 0.132 for the full ridge 3x3 fit.  Neutral pixels pass through unchanged.
wire        sat_O_tlast, sat_O_tuser, sat_O_tvalid;
wire [95:0] sat_O_tdata;
color_saturation_96 #(.SAT_Q3(15)) u_color_saturation_96 (
    .I_clk   (axi4s_video_aclk),
    .I_rst_n (I_rst_n),
    .I_tlast (awb_O_tlast),
    .I_tuser (awb_O_tuser),
    .I_tvalid(awb_O_tvalid),
    .I_tdata (awb_O_tdata),
    .O_tlast (sat_O_tlast),
    .O_tuser (sat_O_tuser),
    .O_tvalid(sat_O_tvalid),
    .O_tdata (sat_O_tdata)
);

wire        curve_O_tlast, curve_O_tuser, curve_O_tvalid;
wire [95:0] curve_O_tdata;
luma_curve_96 #(
    .CURVE_SEL   (CURVE_SEL),
    .CURVE_ENABLE(1),
    .DENOISE_ENABLE(1)
) u_luma_curve_96 (
    .I_clk   (axi4s_video_aclk),
    .I_rst_n (I_rst_n),
    .I_tlast (sat_O_tlast),
    .I_tuser (sat_O_tuser),
    .I_tvalid(sat_O_tvalid),
    .I_tdata (sat_O_tdata),
    .O_tlast (curve_O_tlast),
    .O_tuser (curve_O_tuser),
    .O_tvalid(curve_O_tvalid),
    .O_tdata (curve_O_tdata)
);

// 多模式图像处理流水线，替代原 edge_detect_96。
// 原模块把灰度化 -> 3x3 Sobel -> 幅值 -> 阈值判决全部压在同一拍，
// 实测 23 级逻辑、19.958ns 到达，在 8.888ns 的 MIPI_RX_BYTE_CLK 下
// Setup WNS = -9.261ns。本模块拆成 9 级流水，并支持按键切换算法。
video_proc_96 #(
    .IMG_WIDTH (IMG_WIDTH),
    .IMG_HEIGHT(IMG_HEIGHT),
    .EDGE_TH   (8'd200),
    .GRAY_TH   (8'd128),
    .QUANT_BITS(3)
) u_video_proc_96 (
    .I_clk   (axi4s_video_aclk),
    .I_rst_n (I_rst_n),
    .I_mode  (effective_mode),
    .I_tlast (curve_O_tlast),
    .I_tuser (curve_O_tuser),
    .I_tdata (curve_O_tdata),
    .I_tvalid(curve_O_tvalid),
    .I_tready(O_tready),
    .O_tlast (edge_O_tlast),
    .O_tuser (edge_O_tuser),
    .O_tdata (edge_O_tdata),
    .O_tvalid(edge_O_tvalid),
    .O_tready(edge_O_tready)
);

    data_96bit_to_128bit u_data_96bit_to_128bit(
        .I_clk              ( axi4s_video_aclk  ),
        .I_rst_n            ( I_rst_n           ),

        .I_96b_frame_start  ( edge_O_tuser	    ),
        .I_96b_valid        ( edge_O_tvalid 		),
        .I_96b_data         ( edge_O_tdata		),

        .O_128b_frame_start ( awb_O_tuser_128  	),
        .O_128b_valid       ( awb_O_tvalid_128 	),
        .O_128b_data        ( awb_O_tdata_128  	)
    );



//data96_128 u_data96_128 (
//    .I_tdata(awb_O_tdata),
//    .O_tdata(awb_O_tdata_128)
//);
	
endmodule


//------------------------------------------------------------------------------
// Combinational RAW10 black-level correction for four pixels per beat.
//------------------------------------------------------------------------------
module raw_black_level_corrector #(
    parameter [9:0] BLACK_LEVEL = 10'd64,
    parameter NORMALIZE_FULL_RANGE = 1
)(
    input  wire [39:0] I_tdata,
    output wire [39:0] O_tdata
);

    // Constant-folded Q10 scale; no run-time division. BLACK_LEVEL must be <1023.
    localparam integer SCALE_Q10 = (1023*1024+(1023-BLACK_LEVEL)/2)/(1023-BLACK_LEVEL);

    function [9:0] F_black_correct;
        input [9:0] I_pixel;
        reg [9:0] corrected;
        reg [31:0] scaled;
        begin
            corrected = (I_pixel > BLACK_LEVEL) ? I_pixel - BLACK_LEVEL : 10'd0;
            scaled = corrected * SCALE_Q10 + 32'd512;
            if(!NORMALIZE_FULL_RANGE) F_black_correct = corrected;
            else if(scaled[31:10] > 1023) F_black_correct = 10'd1023;
            else F_black_correct = scaled[19:10];
        end
    endfunction

    assign O_tdata = {
        F_black_correct(I_tdata[39:30]),
        F_black_correct(I_tdata[29:20]),
        F_black_correct(I_tdata[19:10]),
        F_black_correct(I_tdata[ 9: 0])
    };

endmodule


//------------------------------------------------------------------------------
// Pipelined Q8 chart correction.  Input isolation, multiplication,
// accumulation and rounding/clamping are separated by registers: the previous
// single-cycle
// multiply-add path missed the 112.5 MHz setup requirement and produced
// sporadic pixel-bit errors which neighborhood modes amplified into noise.
// ENABLE=0 keeps the same data/control alignment without applying
// the matrix; ENABLE=1 applies the 2026-09-14 fit after AWB, before gamma.
// See tools/chart_fit_20260914.json for all 24 measured patches and coefficients.
//------------------------------------------------------------------------------
module color_correction_matrix #(parameter integer ENABLE=0) (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_tlast,
    input  wire        I_tuser,
    input  wire [95:0] I_tdata,
    input  wire        I_tvalid,
    output reg         O_tlast,
    output reg         O_tuser,
    output reg  [95:0] O_tdata,
    output reg         O_tvalid
);
    reg [95:0] S_data_1d, S_data_2d, S_data_3d;
    reg S_tlast_1d, S_tlast_2d, S_tlast_3d;
    reg S_tuser_1d, S_tuser_2d, S_tuser_3d;
    reg S_tvalid_1d, S_tvalid_2d, S_tvalid_3d;
    // Q8 chart fit with measured neutral-axis correction (2026-09-14).
    reg signed [19:0] red_r [0:3], red_g [0:3], red_b [0:3];
    reg signed [19:0] green_r [0:3], green_g [0:3], green_b [0:3];
    reg signed [19:0] blue_r [0:3], blue_g [0:3], blue_b [0:3];
    reg signed [20:0] red_sum [0:3], green_sum [0:3], blue_sum [0:3];
    integer p;
    function [7:0] clamp_q8;
        input signed [20:0] value;
        reg signed [20:0] rounded;
        begin
            rounded=(value+20'sd128) >>> 8;
            clamp_q8=rounded<0 ? 8'd0 : rounded>255 ? 8'd255 : rounded[7:0];
        end
    endfunction

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_data_1d   <= 96'd0;
            S_data_2d   <= 96'd0;
            S_data_3d   <= 96'd0;
            S_tlast_1d  <= 1'b0;
            S_tlast_2d  <= 1'b0;
            S_tlast_3d  <= 1'b0;
            S_tuser_1d  <= 1'b0;
            S_tuser_2d  <= 1'b0;
            S_tuser_3d  <= 1'b0;
            S_tvalid_1d <= 1'b0;
            S_tvalid_2d <= 1'b0;
            S_tvalid_3d <= 1'b0;
            O_tdata     <= 96'd0;
            O_tlast     <= 1'b0;
            O_tuser     <= 1'b0;
            O_tvalid    <= 1'b0;
            for(p=0;p<4;p=p+1) begin
                red_r[p]<=0; red_g[p]<=0; red_b[p]<=0;
                green_r[p]<=0; green_g[p]<=0; green_b[p]<=0;
                blue_r[p]<=0; blue_g[p]<=0; blue_b[p]<=0;
                red_sum[p]<=0; green_sum[p]<=0; blue_sum[p]<=0;
            end
        end else begin
            S_data_1d   <= I_tdata;
            S_data_2d   <= S_data_1d;
            S_data_3d   <= S_data_2d;
            S_tlast_1d  <= I_tlast;
            S_tlast_2d  <= S_tlast_1d;
            S_tlast_3d  <= S_tlast_2d;
            S_tuser_1d  <= I_tuser;
            S_tuser_2d  <= S_tuser_1d;
            S_tuser_3d  <= S_tuser_2d;
            S_tvalid_1d <= I_tvalid;
            S_tvalid_2d <= S_tvalid_1d;
            S_tvalid_3d <= S_tvalid_2d;
            O_tdata    <= S_data_3d;
            for(p=0;p<4;p=p+1) begin
                // Product stage uses the isolated input register, so no path
                // can run directly from AWB through a multiplier and adder.
                red_r[p] <=  $signed({1'b0,S_data_1d[p*24+16+:8]}) * 20'sd466;
                red_g[p] <= -$signed({1'b0,S_data_1d[p*24+8 +:8]}) * 20'sd36;
                red_b[p] <= -$signed({1'b0,S_data_1d[p*24   +:8]}) * 20'sd110;
                green_r[p] <= -$signed({1'b0,S_data_1d[p*24+16+:8]}) * 20'sd56;
                green_g[p] <=  $signed({1'b0,S_data_1d[p*24+8 +:8]}) * 20'sd337;
                green_b[p] <= -$signed({1'b0,S_data_1d[p*24   +:8]}) * 20'sd28;
                blue_r[p] <= -$signed({1'b0,S_data_1d[p*24+16+:8]}) * 20'sd122;
                blue_g[p] <= -$signed({1'b0,S_data_1d[p*24+8 +:8]}) * 20'sd45;
                blue_b[p] <=  $signed({1'b0,S_data_1d[p*24   +:8]}) * 20'sd368;
                // Accumulation stage: short signed adder tree.
                red_sum[p]   <= red_r[p]   + red_g[p]   + red_b[p];
                green_sum[p] <= green_r[p] + green_g[p] + green_b[p];
                blue_sum[p]  <= blue_r[p]  + blue_g[p]  + blue_b[p];
                // Output stage: Q8 rounding and saturation.
                if(ENABLE) O_tdata[p*24+:24] <= {
                    clamp_q8(red_sum[p]), clamp_q8(green_sum[p]), clamp_q8(blue_sum[p])
                };
            end
            O_tlast    <= S_tlast_3d;
            O_tuser    <= S_tuser_3d;
            O_tvalid   <= S_tvalid_3d;
        end
    end
endmodule


//------------------------------------------------------------------------------
// DSP-free saturation for four RGB888 pixels per beat.
// out = clamp(c + (k-1)*(c-Y)), Y=(R+2G+B)/4, k=SAT_Q3/8 with 8<=SAT_Q3<=15.
// (k-1)*8 is built from three constant shifts, so no multiplier is inferred.
// Three register stages: luma, scaled chroma, rounding/clamp; every sideband
// is delayed by the same three clocks.  Neutral input (R=G=B) is unchanged.
//------------------------------------------------------------------------------
module color_saturation_96 #(parameter integer SAT_Q3 = 15) (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_tlast,
    input  wire        I_tuser,
    input  wire        I_tvalid,
    input  wire [95:0] I_tdata,
    output reg         O_tlast,
    output reg         O_tuser,
    output reg         O_tvalid,
    output reg  [95:0] O_tdata
);
    localparam [2:0] K = SAT_Q3 - 8;
    initial if(SAT_Q3 < 8 || SAT_Q3 > 15) $error("color_saturation_96: SAT_Q3 must be 8..15");
    reg [95:0] s1_data, s2_data;
    reg [39:0] s1_luma;
    reg signed [12:0] s2_chroma [0:11];
    reg [2:0] s1_ctl, s2_ctl;
    integer i;

    function [7:0] clamp_sum;
        input [7:0] c;
        input signed [12:0] e;
        reg signed [13:0] v;
        begin
            v = $signed({6'd0, c}) + ((e + 13'sd4) >>> 3);
            clamp_sum = v < 0 ? 8'd0 : v > 255 ? 8'd255 : v[7:0];
        end
    endfunction

    // (SAT_Q3-8)*(c-Y). At K=7, 8*d-d uses one subtractor per channel.
    function signed [12:0] scaled_chroma;
        input [7:0] c;
        input [9:0] y;
        reg signed [12:0] d;
        begin
            d = $signed({5'd0, c}) - $signed({3'd0, y});
            if(K == 3'd7)
                scaled_chroma = (d <<< 3) - d;
            else
                scaled_chroma = (K[2] ? (d <<< 2) : 13'sd0) +
                                (K[1] ? (d <<< 1) : 13'sd0) +
                                (K[0] ? d : 13'sd0);
        end
    endfunction

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            s1_data <= 96'd0; s2_data <= 96'd0; s1_luma <= 40'd0;
            s1_ctl <= 3'd0; s2_ctl <= 3'd0;
            O_tdata <= 96'd0; O_tlast <= 1'b0; O_tuser <= 1'b0; O_tvalid <= 1'b0;
            for(i=0;i<12;i=i+1) s2_chroma[i] <= 13'sd0;
        end else begin
            s1_data <= I_tdata;
            s1_ctl  <= {I_tlast, I_tuser, I_tvalid};
            for(i=0;i<4;i=i+1)
                s1_luma[i*10+:10] <= ({2'b0,I_tdata[i*24+16+:8]} +
                                      {1'b0,I_tdata[i*24+8+:8],1'b0} +
                                      {2'b0,I_tdata[i*24+:8]} + 10'd2) >> 2;
            s2_data <= s1_data;
            s2_ctl  <= s1_ctl;
            for(i=0;i<12;i=i+1)
                s2_chroma[i] <= scaled_chroma(s1_data[i*8+:8], s1_luma[(i/3)*10+:10]);
            for(i=0;i<12;i=i+1)
                O_tdata[i*8+:8] <= clamp_sum(s2_data[i*8+:8], s2_chroma[i]);
            {O_tlast, O_tuser, O_tvalid} <= s2_ctl;
        end
    end
endmodule
