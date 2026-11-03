// Lossless indexed logo, two synchronous vendor ERAM reads.
// Stage 1: cropped 146x94 image -> 10-bit index; stage 2: original RGB + alpha.
// All 6387 opaque source pixels retained. Entry 0 is transparent.
module anlogic_logo_rom #(
    // INIT_FILE 相对路径基准为源文件所在目录。
    parameter INDEX_FILE="assets/logo_index.dath",
    parameter PALETTE_FILE="assets/logo_palette.dath"
)(
    input wire I_clk,I_rst_n,
    input wire [11:0] I_x,I_y,
    output wire [24:0] O_pixel
);
    wire in_crop=I_x>=12'd7 && I_x<=12'd152 && I_y>=12'd34 && I_y<=12'd127;
    wire [13:0] row={2'b00,I_y}-14'd34;
    wire [13:0] column={2'b00,I_x}-14'd7;
    wire [13:0] addr=in_crop ? (row<<7)+(row<<4)+(row<<1)+column : 14'd16383;
    wire [9:0] index;
    PH1P_LOGIC_ERAM #(
        .DATA_WIDTH_A(10),.DATA_WIDTH_B(10),.ADDR_WIDTH_A(14),.ADDR_WIDTH_B(14),
        .DATA_DEPTH_A(16384),.DATA_DEPTH_B(16384),.MODE("SP"),
        .REGMODE_A("NOREG"),.REGMODE_B("NOREG"),.IMPLEMENT("20K"),
        .RESETMODE_A("ASYNC"),.OREGSET_A("RESET"),.ASYNC_RESET_RELEASE_A("ASYNC"),
        .INIT_FILE(INDEX_FILE)
    ) u_index(
        .clka(I_clk),.cea(1'b1),.ocea(1'b1),.rsta(!I_rst_n),
        .wea(1'b0),.bea(1'b0),.dia(10'd0),.addra(addr),.doa(index),
        .clkb(I_clk),.ceb(1'b0),.oceb(1'b0),.rstb(1'b0),
        .web(1'b0),.beb(1'b0),.dib(10'd0),.addrb(14'd0),.dob(),
        .ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0),.ecc_sbiterr(),.ecc_dbiterr()
    );
    PH1P_LOGIC_ERAM #(
        .DATA_WIDTH_A(25),.DATA_WIDTH_B(25),.ADDR_WIDTH_A(10),.ADDR_WIDTH_B(10),
        .DATA_DEPTH_A(1024),.DATA_DEPTH_B(1024),.MODE("SP"),
        .REGMODE_A("NOREG"),.REGMODE_B("NOREG"),.IMPLEMENT("20K"),
        .RESETMODE_A("ASYNC"),.OREGSET_A("RESET"),.ASYNC_RESET_RELEASE_A("ASYNC"),
        .INIT_FILE(PALETTE_FILE)
    ) u_palette(
        .clka(I_clk),.cea(1'b1),.ocea(1'b1),.rsta(!I_rst_n),
        .wea(1'b0),.bea(1'b0),.dia(25'd0),.addra(index),.doa(O_pixel),
        .clkb(I_clk),.ceb(1'b0),.oceb(1'b0),.rstb(1'b0),
        .web(1'b0),.beb(1'b0),.dib(25'd0),.addrb(10'd0),.dob(),
        .ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0),.ecc_sbiterr(),.ecc_dbiterr()
    );
endmodule
