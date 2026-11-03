//------------------------------------------------------------------------------
// video_proc_96 : 多模式实时图像处理流水线（4 像素并行）
//
// 替代官方 edge_detect_96.v。两个目的：
//   1) 修时序 —— 官方实现把灰度化、3x3 Sobel、幅值、阈值判决全部压在同一拍，
//      实测逻辑级数 22 级（14 个加法器），在 MIPI_RX_BYTE_CLK 112.5MHz
//      (周期 8.888ns) 下 Setup WNS = -8.286ns，1544 个失败端点。
//      本模块拆成 9 级流水，每级只保留很浅的逻辑。
//   2) 多算法可切换 —— 赛题要求最终设计整合为一个 bitstream，不能靠多次
//      烧录分别演示，故用板载按键轮询切换处理模式。
//
// 流水线结构（借鉴本团队 exp6 项目 video_proc.v 的分级思路）：
//   R1  输入寄存、解包 4 个像素
//   R2  亮度乘法      77*R / 150*G / 29*B（每像素 3 个积，共 12 个）
//   R3  亮度求和取高位 -> 4 个 8bit 灰度
//   R4  行缓存读出     取得上一行、上两行的同列组
//   R5  组滑窗寄存     保存 prev2/prev/curr 三组，构成跨组的水平邻域
//   R6  行列加权和     Sobel 的 (1,2,1) 部分和
//   R7  Gx / Gy       两个差值
//   R8  幅值           |Gx| + |Gy| 与钳位
//   R9  模式选择与打包输出
//
// 与官方实现的关键差异：
//   * 官方先把一组 4 个像素平均成"组灰度"再做 Sobel，水平分辨率降为 1/4；
//     本模块对 4 个像素各自独立算 Sobel，保留完整水平分辨率。
//     跨组边界的邻居通过保存前两组解决（见 R5）。
//   * 官方灰度用 (R+G+B)/3 且存在 8bit 溢出（实测 255->84、86->0）；
//     本模块用标准亮度权重 Y = (77R + 150G + 29B) >> 8，位宽足够无溢出。
//   * Sobel 系数为 ±1/±2，用移位加实现，不占乘法器。
//
// 2026-10-23: mode 5 reuses the grayscale line RAM for edge-aware causal
// smoothing and luminance posterization; mode 7 adds independently implemented
// frame-stable colour CDF equalization. No full-frame memory is added.
//
// 【像素排列】
//   经 data_96bit_to_128bit 与 video_out 的打包/解包顺序确认：
//   I_tdata[95:72] 是最左像素，[23:0] 是最右像素。
//
// 【流控】
//   与官方 edge_detect_96 保持一致：内部按 I_tvalid 推进，O_tready 直通
//   I_tready，不做反压缓冲（下游 DDR 写通路自带 FIFO）。
//------------------------------------------------------------------------------
module video_proc_96 #(
    parameter IMG_WIDTH  = 1024,
    parameter IMG_HEIGHT = 600,
    parameter EDGE_TH    = 8'd200,   // 首帧/样本不足时的保守阈值
    parameter EDGE_STATS_MIN_SAMPLES = 1024,
    parameter CARTOON_EDGE_TH = 8'd180, // 卡通描边只保留强轮廓
    parameter GRAY_TH    = 8'd128,   // 灰度二值化阈值
    parameter QUANT_BITS = 2         // Legacy instantiation compatibility; mode 7 is now HE.
)(
    input                   I_clk,
    input                   I_rst_n,

    input       [2:0]       I_mode,      // 处理模式（已在本时钟域同步）

    input                   I_tlast,
    input                   I_tuser,
    input       [95:0]      I_tdata,
    input                   I_tvalid,
    input                   I_tready,

    output reg              O_tlast,
    output reg              O_tuser,
    output reg  [95:0]      O_tdata,
    output reg              O_tvalid,
    output                  O_tready
);

    assign O_tready = I_tready;

    localparam GRP_W  = IMG_WIDTH / 4;    // 每行的像素组数
    localparam STAGES = 9;

    localparam [2:0] MODE_PASS      = 3'd0,   // 直通（原始彩色）
                     MODE_GRAY      = 3'd1,   // 灰度
                     MODE_BIN       = 3'd2,   // 灰度二值化
                     MODE_SOBEL_MAG = 3'd3,   // 边缘强度（灰度显示）
                     MODE_SOBEL_BIN = 3'd4,   // 边缘二值化
                     MODE_CARTOON   = 3'd5,   // 保色亮度平滑/量化 + 对齐的因果轮廓
                     MODE_INVERT    = 3'd6,   // 反色
                     MODE_EQUALIZE  = 3'd7;   // 彩色亮度直方图均衡

    integer i;

    //==========================================================================
    // 控制信号延迟链：{tuser, tlast, tvalid} 与数据同步推进 STAGES 拍
    //==========================================================================
    reg [2:0] ctl [1:STAGES];
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            for (i = 1; i <= STAGES; i = i + 1) ctl[i] <= 3'b000;
        end else begin
            ctl[1] <= {I_tuser, I_tlast, I_tvalid};
            for (i = 2; i <= STAGES; i = i + 1) ctl[i] <= ctl[i-1];
        end
    end

    wire vld_s1 = ctl[1][0];

    //==========================================================================
    // 行列计数与边界掩码（在 R1 算，随后延迟到输出级，避免在末级叠加组合逻辑）
    //   这是 exp6 项目踩过的坑：把坐标比较放在模式选择那一级会直接吃掉 WNS。
    //==========================================================================
    reg [11:0] grp_x;      // 行内组序号
    reg [11:0] row_y;      // 行序号

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            grp_x <= 12'd0;
            row_y <= 12'd0;
        end else if (ctl[1][2]) begin          // 帧首也可能落在消隐拍
            grp_x <= (vld_s1 && !ctl[1][1]) ? 12'd1 : 12'd0;
            row_y <= 12'd0;
        end else if (vld_s1) begin
            if (ctl[1][1]) begin               // tlast：行尾
                grp_x <= 12'd0;
                row_y <= (row_y == IMG_HEIGHT-1) ? 12'd0 : row_y + 1'b1;
            end else begin
                grp_x <= grp_x + 1'b1;
            end
        end
    end

    // 3x3 窗口完整的条件。窗口中心是 prev 组 g，横向跨度为 [g-1, g, g+1]：
    //   * 右边界必须挡掉 g = GRP_W-1，否则 g+1 会越到下一行（帧末则越到帧外），
    //     产生虚假边缘——这是仿真中在 (x=GRP_W-1, y=IMG_H-1) 抓到的实际缺陷。
    //   * 左边界留 3 组余量：消隐期间 t_c/t_p1/t_p2 三级移位寄存器会被冲入无效
    //     数据，行首需要几个有效拍才能冲干净。
    //   * 纵向需要上两行都在行缓存里，故 row_y >= 2。
    // 1024 宽共 256 组，屏蔽左 12 像素、右 4 像素，肉眼不可见。
    wire win_ok_s1 = (grp_x >= 12'd3) && (grp_x <= GRP_W-2) && (row_y >= 12'd2);

    reg [2:STAGES] win_ok;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) win_ok <= {(STAGES-1){1'b0}};
        else begin
            win_ok[2] <= win_ok_s1;
            for (i = 3; i <= STAGES; i = i + 1) win_ok[i] <= win_ok[i-1];
        end
    end

    //==========================================================================
    // R1：输入寄存与解包
    //==========================================================================
    reg [7:0] r1_r [0:3];
    reg [7:0] r1_g [0:3];
    reg [7:0] r1_b [0:3];

    always @(posedge I_clk) begin
        r1_r[0] <= I_tdata[95:88];  r1_g[0] <= I_tdata[87:80];  r1_b[0] <= I_tdata[79:72];
        r1_r[1] <= I_tdata[71:64];  r1_g[1] <= I_tdata[63:56];  r1_b[1] <= I_tdata[55:48];
        r1_r[2] <= I_tdata[47:40];  r1_g[2] <= I_tdata[39:32];  r1_b[2] <= I_tdata[31:24];
        r1_r[3] <= I_tdata[23:16];  r1_g[3] <= I_tdata[15: 8];  r1_b[3] <= I_tdata[ 7: 0];
    end

    // 原始彩色延迟链：直通/反色/量化等点运算需要未处理的原像素
    reg [95:0] rgb_dly [1:STAGES];
    always @(posedge I_clk) begin
        rgb_dly[1] <= I_tdata;
        for (i = 2; i <= STAGES; i = i + 1) rgb_dly[i] <= rgb_dly[i-1];
    end

    //==========================================================================
    // R2：亮度乘法  Y = (77R + 150G + 29B) >> 8
    //     每个乘积独占一拍，避免乘加同拍造成逻辑过深（exp6 在此处踩过坑）
    //==========================================================================
    reg [15:0] pr2 [0:3];
    reg [15:0] pg2 [0:3];
    reg [15:0] pb2 [0:3];

    always @(posedge I_clk) begin
        for (i = 0; i < 4; i = i + 1) begin
            pr2[i] <= r1_r[i] * 8'd77;
            pg2[i] <= r1_g[i] * 8'd150;
            pb2[i] <= r1_b[i] * 8'd29;
        end
    end

    //==========================================================================
    // R3：亮度求和并取高 8 位 -> 灰度
    //==========================================================================
    reg [7:0] gray3 [0:3];
    always @(posedge I_clk) begin
        for (i = 0; i < 4; i = i + 1) begin
            gray3[i] <= (pr2[i] + pg2[i] + pb2[i]) >> 8;
        end
    end

    wire [31:0] gray_grp3 = {gray3[0], gray3[1], gray3[2], gray3[3]};

    // Point modes use the current pixel. The Sobel centre below is the
    // previous row, so reusing it for gray/bin introduces a one-row offset
    // and makes those modes depend on the neighbourhood-valid mask.
    reg [31:0] gray_direct [4:9];
    always @(posedge I_clk) begin
        gray_direct[4] <= gray_grp3;
        for (i = 5; i <= 9; i = i + 1)
            gray_direct[i] <= gray_direct[i-1];
    end

    //==========================================================================
    // R4：行缓存读出（上一行、上两行的同列组）
    //==========================================================================
    wire [31:0] row_m1, row_m2;

    line_buffer_grp #(.DW(32), .MAX_GRPS(512)) u_lb (
        .clk    (I_clk),
        .rst_n  (I_rst_n),
        .vld    (ctl[3][0]),
        .sol    (ctl[3][2] | ctl[4][1]),   // 帧首/行尾后一拍复位；行缓存会在消隐期间保持地址0
        .din    (gray_grp3),
        .row_m1 (row_m1),
        .row_m2 (row_m2)
    );

    // 行缓存输出比 din 晚 1 拍，把当前行也延迟 1 拍对齐
    reg [31:0] row_m0;
    always @(posedge I_clk) row_m0 <= gray_grp3;

    //==========================================================================
    // R5：组滑窗寄存
    //   三行各保存 prev2 / prev / curr 三组。计算的是 prev 这一组的 4 个像素，
    //   其左邻居来自 prev2 的最后一个像素，右邻居来自 curr 的第一个像素。
    //   这样即可对每个像素独立做 Sobel，而不必像官方实现那样先做组平均。
    //==========================================================================
    reg [31:0] t_p2, t_p1, t_c;   // top    行 (row_m2)
    reg [31:0] m_p2, m_p1, m_c;   // middle 行 (row_m1)
    reg [31:0] b_p2, b_p1, b_c;   // bottom 行 (row_m0，当前行)

    always @(posedge I_clk) begin
        t_p2 <= t_p1;  t_p1 <= t_c;  t_c <= row_m2;
        m_p2 <= m_p1;  m_p1 <= m_c;  m_c <= row_m1;
        b_p2 <= b_p1;  b_p1 <= b_c;  b_c <= row_m0;
    end

    // 每行取出 6 个相邻灰度：[prev2的最右, prev的4个, curr的最左]
    // 组内位序：[31:24]=最左像素, [7:0]=最右像素
    wire [7:0] ts [0:5];
    wire [7:0] ms [0:5];
    wire [7:0] bs [0:5];

    assign ts[0] = t_p2[ 7: 0];  assign ts[1] = t_p1[31:24];
    assign ts[2] = t_p1[23:16];  assign ts[3] = t_p1[15: 8];
    assign ts[4] = t_p1[ 7: 0];  assign ts[5] = t_c [31:24];

    assign ms[0] = m_p2[ 7: 0];  assign ms[1] = m_p1[31:24];
    assign ms[2] = m_p1[23:16];  assign ms[3] = m_p1[15: 8];
    assign ms[4] = m_p1[ 7: 0];  assign ms[5] = m_c [31:24];

    assign bs[0] = b_p2[ 7: 0];  assign bs[1] = b_p1[31:24];
    assign bs[2] = b_p1[23:16];  assign bs[3] = b_p1[15: 8];
    assign bs[4] = b_p1[ 7: 0];  assign bs[5] = b_c [31:24];

    //==========================================================================
    // R6：Sobel 的行/列加权部分和 (1,2,1)
    //   Gx = (右列加权和) - (左列加权和)
    //   Gy = (上行加权和) - (下行加权和)
    //   系数均为 1/2，用移位加实现，不占乘法器
    //==========================================================================
    reg [10:0] col_l [0:3];   // 左列: ts[i] + 2*ms[i] + bs[i]
    reg [10:0] col_r [0:3];   // 右列: ts[i+2] + 2*ms[i+2] + bs[i+2]
    reg [10:0] row_t [0:3];   // 上行: ts[i] + 2*ts[i+1] + ts[i+2]
    reg [10:0] row_b [0:3];   // 下行: bs[i] + 2*bs[i+1] + bs[i+2]

    always @(posedge I_clk) begin
        for (i = 0; i < 4; i = i + 1) begin
            col_l[i] <= ts[i]   + (ms[i]   << 1) + bs[i];
            col_r[i] <= ts[i+2] + (ms[i+2] << 1) + bs[i+2];
            row_t[i] <= ts[i]   + (ts[i+1] << 1) + ts[i+2];
            row_b[i] <= bs[i]   + (bs[i+1] << 1) + bs[i+2];
        end
    end

    //==========================================================================
    // R7：Gx / Gy
    //==========================================================================
    reg signed [11:0] gx7 [0:3];
    reg signed [11:0] gy7 [0:3];

    always @(posedge I_clk) begin
        for (i = 0; i < 4; i = i + 1) begin
            gx7[i] <= $signed({1'b0, col_r[i]}) - $signed({1'b0, col_l[i]});
            gy7[i] <= $signed({1'b0, row_t[i]}) - $signed({1'b0, row_b[i]});
        end
    end

    //==========================================================================
    // R8：幅值 |Gx| + |Gy| 并钳位到 8bit
    //   用 |Gx|+|Gy| 近似 sqrt(Gx^2+Gy^2)，最大误差约 12%，
    //   省掉平方和开方，对"检测边缘"完全够用。
    //==========================================================================
    wire [11:0] abs_gx [0:3];
    wire [11:0] abs_gy [0:3];
    genvar gi;
    generate
        for (gi = 0; gi < 4; gi = gi + 1) begin : g_abs
            assign abs_gx[gi] = gx7[gi][11] ? (~gx7[gi] + 1'b1) : gx7[gi];
            assign abs_gy[gi] = gy7[gi][11] ? (~gy7[gi] + 1'b1) : gy7[gi];
        end
    endgenerate

    // Independent cartoon treatment: reuse the GRAY line window, never add
    // full-colour line/frame RAM. At the current pixel, gently average left
    // and above only when their luminance differs by <=12. Strong transitions
    // retain the centre value. Quantize the filtered Y to rounded 32-step
    // bands, then apply ONE signed offset to R/G/B (not three channel bins).
    // The causal edge is measured at the SAME current pixel as the colour;
    // overlaying the old previous-row Sobel on current RGB shifted outlines.
    function [7:0] cartoon_clip;
        input signed [9:0] value;
        begin cartoon_clip=value<0 ? 8'd0 : value>255 ? 8'd255 : value[7:0]; end
    endfunction
    function signed [9:0] cartoon_delta;
        input [9:0] filtered_sum;
        input [7:0] centre;
        reg [8:0] rounded;
        reg [7:0] target;
        begin
            rounded={1'b0,filtered_sum[9:2]}+9'd16;
            target=rounded[8] ? 8'd255 : {rounded[7:5],5'd0};
            cartoon_delta=$signed({1'b0,target})-$signed({1'b0,centre});
        end
    endfunction
    reg [9:0] cartoon_sum6 [0:3];
    reg [7:0] cartoon_centre6 [0:3];
    reg [8:0] cartoon_gradient6 [0:3];
    reg cartoon_window6;
    reg signed [9:0] cartoon_offset7 [0:3];
    reg [3:0] cartoon_edge7;
    reg [95:0] cartoon_rgb8;
    generate for(gi=0;gi<4;gi=gi+1) begin : g_cartoon
        wire [7:0] centre=gray_direct[6][31-8*gi -:8];
        wire [7:0] left=bs[gi],above=ms[gi+1];
        wire [7:0] dl=centre>=left ? centre-left : left-centre;
        wire [7:0] da=centre>=above ? centre-above : above-centre;
        wire [7:0] near_l=dl<=12 ? left : centre;
        wire [7:0] near_a=da<=12 ? above : centre;
        always @(posedge I_clk) begin
            cartoon_centre6[gi]<=centre;
            cartoon_sum6[gi]<=win_ok[6] ?
                {1'b0,centre,1'b0}+{2'b0,near_l}+{2'b0,near_a}+10'd2 :
                {centre,2'b00};
            cartoon_gradient6[gi]<={1'b0,dl}+{1'b0,da};
            cartoon_offset7[gi]<=cartoon_delta(cartoon_sum6[gi],cartoon_centre6[gi]);
            cartoon_edge7[gi]<=cartoon_window6 && cartoon_gradient6[gi]>(CARTOON_EDGE_TH/3);
            cartoon_rgb8[95-24*gi -:24]<=cartoon_edge7[gi] ? 24'd0 : {
                cartoon_clip($signed({1'b0,rgb_dly[8][95-24*gi -:8]})+cartoon_offset7[gi]),
                cartoon_clip($signed({1'b0,rgb_dly[8][87-24*gi -:8]})+cartoon_offset7[gi]),
                cartoon_clip($signed({1'b0,rgb_dly[8][79-24*gi -:8]})+cartoon_offset7[gi])};
        end
    end endgenerate
    always @(posedge I_clk) cartoon_window6<=win_ok[6];

    // HE takes the existing stage-four RGB/control tap, then its five clocks
    // reach stage nine directly. Its rgb1..3 registers now duplicate existing
    // rgb_dly[5..7], instead of requiring another four full RGB delay stages.
    // Shift reset release by the same four clocks: even a first frame marker
    // adjacent to histogram clearing sees the exact previous HE state sequence.
    reg [3:0] he_release;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) he_release<=4'b0000;
        else he_release<={he_release[2:0],1'b1};
    end
    wire [95:0] equalized9;
    luma_histogram_equalizer_96 #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) u_he(
        .clk(I_clk),.rst_n(he_release[3]),.enable(1'b1),.data_in(rgb_dly[4]),
        .valid_in(ctl[4][0]),.last_in(ctl[4][1]),.user_in(ctl[4][2]),
        .data_out(equalized9),.valid_out(),.last_out(),.user_out());

    reg [7:0] mag8 [0:3];
    always @(posedge I_clk) begin
        for (i = 0; i < 4; i = i + 1) begin
            mag8[i] <= ((abs_gx[i] + abs_gy[i]) > 12'd255)
                       ? 8'd255 : (abs_gx[i] + abs_gy[i]);
        end
    end

    // One Sobel sample per 16 four-pixel groups builds a coarse frame median.
    // The threshold changes only at frame start, so no row can switch halfway.
    // Sampling while any mode is displayed lets edge binary adapt immediately
    // after the first complete frame following a mode change.
    reg [3:0] edge_sample_div;
    reg [15:0] edge_samples, edge_gt32, edge_gt64, edge_gt128, edge_gt192;
    reg [7:0] edge_th_active;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            edge_sample_div <= 4'd0;
            edge_samples <= 16'd0;
            edge_gt32 <= 16'd0;
            edge_gt64 <= 16'd0;
            edge_gt128 <= 16'd0;
            edge_gt192 <= 16'd0;
            edge_th_active <= EDGE_TH;
        end else if (ctl[STAGES][2]) begin
            if (edge_samples >= EDGE_STATS_MIN_SAMPLES) begin
                if (edge_gt192 > (edge_samples >> 1)) edge_th_active <= 8'd248;
                else if (edge_gt128 > (edge_samples >> 1)) edge_th_active <= 8'd232;
                else if (edge_gt64 > (edge_samples >> 1)) edge_th_active <= 8'd208;
                else if (edge_gt32 > (edge_samples >> 1)) edge_th_active <= 8'd184;
                else edge_th_active <= 8'd96;
            end
            edge_sample_div <= 4'd0;
            edge_samples <= 16'd0;
            edge_gt32 <= 16'd0;
            edge_gt64 <= 16'd0;
            edge_gt128 <= 16'd0;
            edge_gt192 <= 16'd0;
        end else if (ctl[STAGES][0]) begin
            edge_sample_div <= edge_sample_div + 1'b1;
            if (edge_sample_div == 4'd0) begin
                edge_samples <= edge_samples + 1'b1;
                if (mag8[0] > 8'd32) edge_gt32 <= edge_gt32 + 1'b1;
                if (mag8[0] > 8'd64) edge_gt64 <= edge_gt64 + 1'b1;
                if (mag8[0] > 8'd128) edge_gt128 <= edge_gt128 + 1'b1;
                if (mag8[0] > 8'd192) edge_gt192 <= edge_gt192 + 1'b1;
            end
        end
    end

    // 注意：这里不能再多打一拍。逐级核算的延迟（以输入拍为单位）为
    // col_* = 7, gx7/gy7 = 8, mag8 = 9；直通灰度 gray_direct[9] 也到第 9 级。

    //==========================================================================
    // R9：模式选择与打包输出
    //   本级只做"选择"，所有比较/运算都已在前级完成，保证末级组合逻辑很浅。
    //==========================================================================
    wire [95:0] rgb_out = rgb_dly[STAGES];   // 与输出同拍的原始彩色

    // 模式信号复制 4 份，每份只驱动一个像素通道的 24 个 mux。
    // 不复制的话 I_mode 一根线要驱动 96 个 mux，实测本工程 HDMI 域就是
    // 被一根 fanout=491 的网络拖出 5.2ns 的走线延迟，这里提前避掉。
    reg [2:0] mode_r [0:3]; //synthesis syn_preserve = 1
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            for (i = 0; i < 4; i = i + 1) mode_r[i] <= 3'd0;
        else
            for (i = 0; i < 4; i = i + 1) mode_r[i] <= I_mode;
    end

    // 各模式下单个像素的结果（组合，但每项都很浅）
    reg  [23:0] px [0:3];
    integer k;
    always @(*) begin
        for (k = 0; k < 4; k = k + 1) begin
            case (mode_r[k])
                MODE_GRAY:
                    px[k] = {3{gray_direct[9][31-8*k -: 8]}};
                MODE_BIN:
                    px[k] = (gray_direct[9][31-8*k -: 8] > GRAY_TH) ? 24'hFFFFFF : 24'h000000;
                MODE_SOBEL_MAG:
                    px[k] = {3{mag8[k]}};
                MODE_SOBEL_BIN:
                    px[k] = (mag8[k] > edge_th_active) ? 24'hFFFFFF : 24'h000000;
                MODE_CARTOON:
                    px[k] = cartoon_rgb8[95-24*k -:24];
                MODE_INVERT:
                    px[k] = ~rgb_out[95-24*k -: 24];
                MODE_EQUALIZE:
                    px[k] = equalized9[95-24*k -:24];
                default:                                  // MODE_PASS
                    px[k] = rgb_out[95-24*k -: 24];
            endcase
        end
    end

    // Sobel 在窗口不完整处输出黑；灰度/二值/均衡等点运算不受影响，
    // 卡通模式在窗口不完整处仍显示量化色块。
    // 这个判断整体提前一级算好并寄存，末级只剩一个与门：
    //   px_ok_r 在本拍的值 = 上一拍算出的 win_ok[STAGES-1]，正好等于 win_ok[STAGES]，
    //   与 mag8 对齐。同样复制 4 份降 fanout。
    wire need_window = (I_mode == MODE_SOBEL_MAG) || (I_mode == MODE_SOBEL_BIN);

    reg [3:0] px_ok_r; //synthesis syn_preserve = 1
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) px_ok_r <= 4'b0000;
        else          px_ok_r <= {4{(!need_window) || win_ok[STAGES-1]}};
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_tvalid <= 1'b0;
            O_tuser  <= 1'b0;
            O_tlast  <= 1'b0;
            O_tdata  <= 96'd0;
        end else begin
            O_tvalid <= ctl[STAGES][0];
            O_tlast  <= ctl[STAGES][1];
            O_tuser  <= ctl[STAGES][2];

            if (ctl[STAGES][0])
                O_tdata <= { px_ok_r[0] ? px[0] : 24'd0,
                             px_ok_r[1] ? px[1] : 24'd0,
                             px_ok_r[2] ? px[2] : 24'd0,
                             px_ok_r[3] ? px[3] : 24'd0 };
            else
                O_tdata <= 96'd0;
        end
    end

endmodule
