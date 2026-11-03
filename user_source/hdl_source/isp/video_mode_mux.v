//------------------------------------------------------------------------------
// video_mode_mux —— 多算法输出选择与卡通化混合
//
// 赛题要求最终设计必须整合为一个 bitstream，不能靠多次烧录分别演示各项功能，
// 因此用板载拨码开关在同一套设计内实时切换处理算法。
//
//   I_mode  显示内容                        对应赛题
//   ------  --------------------------      ----------------------
//   2'b00   原始彩色实时图像                基础要求 1、3
//   2'b01   边缘检测二值图（白边黑底）      基础要求 2
//   2'b10   卡通化（量化色块 + 黑色描边）   扩展要求 1
//   2'b11   预留（目标检测）                扩展要求 2
//
// 三路输入必须已经相位对齐：
//   edge_detect_96 与 color_quant_96 均为 1 拍输出延迟，故直接对齐。
//
// I_mode 来自板载拨码开关，属异步输入，内部做两级同步后再使用，
// 避免亚稳态在切换瞬间打乱视频流。
//------------------------------------------------------------------------------
module video_mode_mux (
    input               I_clk,
    input               I_rst_n,

    input      [1:0]    I_mode,           // 异步输入（拨码开关）

    input      [95:0]   I_raw_tdata,      // 原始彩色（已对齐）
    input      [95:0]   I_quant_tdata,    // 量化彩色（已对齐）
    input      [95:0]   I_edge_tdata,     // 边缘二值（已对齐）

    input               I_tlast,
    input               I_tuser,
    input               I_tvalid,

    output reg          O_tlast,
    output reg          O_tuser,
    output reg          O_tvalid,
    output reg [95:0]   O_tdata
);

    //--------------------------------------------------------------------------
    // 拨码开关跨时钟域同步（两级触发器）
    //--------------------------------------------------------------------------
    reg [1:0] mode_meta;
    reg [1:0] mode_sync;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            mode_meta <= 2'b00;
            mode_sync <= 2'b00;
        end else begin
            mode_meta <= I_mode;
            mode_sync <= mode_meta;
        end
    end

    //--------------------------------------------------------------------------
    // 卡通化混合：边缘处画黑线，非边缘处填量化色块
    // edge_detect_96 输出的每个像素非纯白即纯黑，取其最高位即可判别是否为边缘
    //--------------------------------------------------------------------------
    localparam [23:0] EDGE_LINE_COLOR = 24'h000000;   // 描边颜色

    wire e0 = I_edge_tdata[95];
    wire e1 = I_edge_tdata[71];
    wire e2 = I_edge_tdata[47];
    wire e3 = I_edge_tdata[23];

    wire [95:0] cartoon_comb = {
        e0 ? EDGE_LINE_COLOR : I_quant_tdata[95:72],
        e1 ? EDGE_LINE_COLOR : I_quant_tdata[71:48],
        e2 ? EDGE_LINE_COLOR : I_quant_tdata[47:24],
        e3 ? EDGE_LINE_COLOR : I_quant_tdata[23: 0]
    };

    //--------------------------------------------------------------------------
    // 输出寄存器化：ISP 时钟域时序本就紧张，此处打拍避免把选择逻辑
    // 串进已有的关键路径
    //--------------------------------------------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_tlast  <= 1'b0;
            O_tuser  <= 1'b0;
            O_tvalid <= 1'b0;
            O_tdata  <= 96'd0;
        end else begin
            O_tlast  <= I_tlast;
            O_tuser  <= I_tuser;
            O_tvalid <= I_tvalid;

            if (I_tvalid) begin
                case (mode_sync)
                    2'b00:   O_tdata <= I_raw_tdata;
                    2'b01:   O_tdata <= I_edge_tdata;
                    2'b10:   O_tdata <= cartoon_comb;
                    default: O_tdata <= I_raw_tdata;   // 2'b11 暂未实现，回落到原图
                endcase
            end else begin
                O_tdata <= 96'd0;
            end
        end
    end

endmodule
