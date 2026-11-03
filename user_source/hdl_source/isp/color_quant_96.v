//------------------------------------------------------------------------------
// color_quant_96 —— 色彩量化（画面卡通化的基础）
//
// 作用：
//   1. 对 96bit(4像素 RGB888) 做色彩量化，每通道只保留高 KEEP_BITS 位，
//      使连续渐变变成大色块，产生海报/卡通风格。
//   2. 同时输出一路"仅打拍、不做处理"的原始彩色，用于与 edge_detect_96
//      的 1 拍延迟对齐（edge_detect_96 输出寄存器化，延迟恰为 1 拍）。
//
// 量化算法：
//   直接截断低位会让亮度整体变暗（255 -> 224），所以截断后把高位循环回填到
//   低位，保证 0->0、255->255，量化前后动态范围一致。
//   例 KEEP_BITS=3: 0xFF -> 0xE0 -> 0xE0|0x1C|0x03 = 0xFF
//
// 硬件代价：
//   纯位运算与常数移位，综合后是连线+少量或门，无乘除、无进位链，
//   不会加深关键路径（本设计 ISP 域时序本就紧张，必须避免引入组合深度）。
//------------------------------------------------------------------------------
module color_quant_96 #(
    parameter KEEP_BITS = 3          // 每通道保留的高位数：2/3/4，越小色块越大
)(
    input               I_clk,
    input               I_rst_n,

    input               I_tlast,
    input               I_tuser,
    input      [95:0]   I_tdata,
    input               I_tvalid,

    output reg          O_tlast,
    output reg          O_tuser,
    output reg          O_tvalid,
    output reg [95:0]   O_raw_tdata,     // 原始彩色，打1拍
    output reg [95:0]   O_quant_tdata    // 量化彩色，打1拍
);

    localparam LOW_BITS = 8 - KEEP_BITS;

    // 单通道量化：截断低位后，用高位循环回填，保持满量程
    function [7:0] quant_ch;
        input [7:0] c;
        reg   [7:0] hi;
        begin
            hi       = c & ({8{1'b1}} << LOW_BITS);
            quant_ch = hi | (hi >> KEEP_BITS) | (hi >> (2*KEEP_BITS));
        end
    endfunction

    // 单像素量化：RGB 三通道各自量化
    function [23:0] quant_pix;
        input [23:0] p;
        begin
            quant_pix = { quant_ch(p[23:16]),
                          quant_ch(p[15: 8]),
                          quant_ch(p[ 7: 0]) };
        end
    endfunction

    wire [95:0] quant_comb = { quant_pix(I_tdata[95:72]),
                               quant_pix(I_tdata[71:48]),
                               quant_pix(I_tdata[47:24]),
                               quant_pix(I_tdata[23: 0]) };

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_tlast      <= 1'b0;
            O_tuser      <= 1'b0;
            O_tvalid     <= 1'b0;
            O_raw_tdata  <= 96'd0;
            O_quant_tdata<= 96'd0;
        end else begin
            O_tlast      <= I_tlast;
            O_tuser      <= I_tuser;
            O_tvalid     <= I_tvalid;
            if (I_tvalid) begin
                O_raw_tdata   <= I_tdata;
                O_quant_tdata <= quant_comb;
            end else begin
                O_raw_tdata   <= 96'd0;
                O_quant_tdata <= 96'd0;
            end
        end
    end

endmodule
