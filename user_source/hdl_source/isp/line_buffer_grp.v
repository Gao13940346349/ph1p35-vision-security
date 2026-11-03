//------------------------------------------------------------------------------
// line_buffer_grp : 两行行缓存（按"像素组"为单位）
//
// 结构与时序处理方式移植自本团队 exp6 项目的 line_buffer.v，
// 区别是本设计 MIPI 通路一拍并行 4 个像素，故缓存单元宽度为
// 4 x 8bit = 32bit（一组 4 个灰度），深度按组数计算。
//
// 【为什么只需要两行】
//   3x3 窗口要同时看到第 y-1、y、y+1 三行。当前正在输入的就是第 y+1 行，
//   所以只需把前两行存下来，第三行是实时进来的。
//
// 【容量】
//   1024 宽 -> 256 组 x 32bit x 2 行 = 16.4kbit，PH1P35 有 2160kbit ERAM，
//   占用可忽略。若缓存 24bit 彩色则要多 3 倍。
//
// 【最容易写错的地方】
//   BRAM 读出打一拍，所以 buf1 的写地址必须相应延迟一拍才能对齐，
//   否则会错位一个组（4 个像素）。
//
// 【输出时序】
//   row_m1 / row_m2 比输入 din 晚 1 个时钟，使用方需把 din 也延迟 1 拍
//   再一起组窗口。
//------------------------------------------------------------------------------
module line_buffer_grp #(
    parameter DW       = 32,      // 一组像素的灰度总位宽 (4 x 8bit)
    parameter MAX_GRPS = 512      // 支持的最大组数，需 >= IMG_WIDTH/4
)(
    input  wire          clk,
    input  wire          rst_n,

    input  wire          vld,     // 本拍数据有效
    input  wire          sol,     // 行首复位，可在消隐拍提前到达，也可与首个有效组同拍
    input  wire [DW-1:0] din,     // 当前行的一组灰度（第 y+1 行）

    output wire [DW-1:0] row_m1,  // 上一行同列组（第 y 行），比 din 晚 1 拍
    output wire [DW-1:0] row_m2   // 上两行同列组（第 y-1 行），比 din 晚 1 拍
);

    localparam AW = $clog2(MAX_GRPS);

    (* ram_style = "block" *) reg [DW-1:0] buf0 [0:MAX_GRPS-1];
    (* ram_style = "block" *) reg [DW-1:0] buf1 [0:MAX_GRPS-1];

    reg [AW-1:0] wr_addr;
    reg [AW-1:0] addr_d;
    reg          vld_d;
    reg [DW-1:0] q0, q1;

    // 行首复位不能受 vld 限制：上游行尾后一拍的 sol 通常落在消隐中。
    // 若此时忽略 sol，1280 宽的 320 组就会按 RAM 的 512 地址循环，
    // 上一行读出在 x=(512-320)*4=768 处发生左右拼接。
    // 消隐时先把下一组地址保持为 0；若 sol 与有效组同拍，该组使用地址 0，
    // 下一组地址为 1。RAM 本身仍只在有效拍读写，不在消隐中清空数据。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            wr_addr <= {AW{1'b0}};
        else if (sol)
            wr_addr <= vld ? {{(AW-1){1'b0}}, 1'b1} : {AW{1'b0}};
        else if (vld)
            wr_addr <= wr_addr + 1'b1;
    end

    // 本拍实际使用的地址：行首强制为 0
    wire [AW-1:0] cur_addr = sol ? {AW{1'b0}} : wr_addr;

    // 地址与有效标志延迟一拍，供 buf1 写入对齐
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            addr_d <= {AW{1'b0}};
            vld_d  <= 1'b0;
        end else begin
            addr_d <= cur_addr;
            vld_d  <= vld;
        end
    end

    // buf0：读出上一行该列组，同时写入当前行
    // buf1：读出上两行该列组；写入上一拍从 buf0 读出的值（地址已对齐）
    always @(posedge clk) begin
        if (vld) begin
            q0 <= buf0[cur_addr];
            q1 <= buf1[cur_addr];
            buf0[cur_addr] <= din;
        end
        if (vld_d) begin
            buf1[addr_d] <= q0;
        end
    end

    assign row_m1 = q0;
    assign row_m2 = q1;

endmodule
