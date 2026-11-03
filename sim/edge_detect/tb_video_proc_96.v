`timescale 1ns/1ps
//------------------------------------------------------------------------------
// video_proc_96 多模式流水线功能仿真
//
// 验证点：
//   P1 流控：输出有效拍数 == 输入有效拍数（流水线不吞不吐）
//   P2 帧/行标记 tuser/tlast 逐拍对齐透传
//   G1 灰度模式：均匀彩色图 -> 输出为对应亮度的灰阶，且无 8bit 溢出
//   B1 二值化：亮度高于/低于阈值分别输出白/黑
//   S1 边缘检测：均匀图无虚假边缘
//   S2 边缘检测：垂直边缘能检出，且位置合理
//   S3 边缘检测：水平边缘能检出（验证行缓存与 Gy 通路）
//   N1 直通模式：输出与输入逐位一致
//   I1 反色模式：输出为输入按位取反
//------------------------------------------------------------------------------
module tb_video_proc_96 #(
    parameter IMG_WIDTH = 32,
    parameter IMG_HEIGHT = 10
);
    localparam GRP        = IMG_WIDTH/4;    // 8 组/行
    localparam EDGE_TH    = 8'd60;
    localparam GRAY_TH    = 8'd128;

    localparam [2:0] M_PASS=0, M_GRAY=1, M_BIN=2, M_SMAG=3,
                     M_SBIN=4, M_CART=5, M_INV=6, M_HEQ=7;

    localparam STAGES_WAIT = 15;   // 等流水线排空的拍数（>9级流水深度）
    localparam SAMPLE_X    = GRP-4; // Also exercises columns beyond the old 1024 width.
    localparam SAMPLE_Y    = 5;

    reg         clk = 1'b0;
    reg         rst_n;
    reg  [2:0]  mode;
    reg         I_tlast, I_tuser, I_tvalid;
    reg  [95:0] I_tdata;
    wire        O_tlast, O_tuser, O_tvalid, O_tready;
    wire [95:0] O_tdata;

    video_proc_96 #(
        .IMG_WIDTH (IMG_WIDTH),
        .IMG_HEIGHT(IMG_HEIGHT),
        .EDGE_TH   (EDGE_TH),
        .GRAY_TH   (GRAY_TH),
        .QUANT_BITS(3)
    ) dut (
        .I_clk(clk), .I_rst_n(rst_n), .I_mode(mode),
        .I_tlast(I_tlast), .I_tuser(I_tuser), .I_tdata(I_tdata),
        .I_tvalid(I_tvalid), .I_tready(1'b1),
        .O_tlast(O_tlast), .O_tuser(O_tuser), .O_tdata(O_tdata),
        .O_tvalid(O_tvalid), .O_tready(O_tready)
    );

    always #5 clk = ~clk;

    integer errors;
    integer in_beats, out_beats, out_user, out_last;
    integer white_cnt, black_cnt;
    integer wx_min, wx_max, wy_min, wy_max;
    integer oidx, ox, oy;
    reg     counting;
    reg [95:0] last_out;
    reg [95:0] samp_out;

    always @(posedge clk) begin
        if (counting && O_tvalid) begin
            ox = oidx % GRP;
            oy = oidx / GRP;
            if (O_tuser !== (oidx==0) || O_tlast !== (ox==GRP-1))
                $fatal(1,"Misaligned control at group %0d in mode %0d",oidx,mode);
            if (^O_tdata === 1'bx) $fatal(1,"Unknown output in mode %0d",mode);
            out_beats = out_beats + 1;
            if (O_tuser) out_user = out_user + 1;
            if (O_tlast) out_last = out_last + 1;
            if (O_tdata[23:0] == 24'hFFFFFF) begin
                white_cnt = white_cnt + 1;
                if (ox < wx_min) wx_min = ox;
                if (ox > wx_max) wx_max = ox;
                if (oy < wy_min) wy_min = oy;
                if (oy > wy_max) wy_max = oy;
            end
            if (O_tdata[23:0] == 24'h000000) black_cnt = black_cnt + 1;
            last_out = O_tdata;
            // 采样帧中间一个确定不在边界屏蔽区内的位置，用于点运算类模式的数值校验
            if (ox == SAMPLE_X && oy == SAMPLE_Y) samp_out = O_tdata;
            oidx = oidx + 1;
        end
    end

    // 构造一组 4 个同色像素
    function [95:0] mk;
        input [7:0] r; input [7:0] g; input [7:0] b;
        begin mk = {4{r,g,b}}; end
    endfunction

    // 图案：0=均匀灰128  1=垂直边缘(左黑右白)  2=水平边缘(上黑下白)  3=均匀(200,100,50)
    function [95:0] pattern;
        input integer mode_p;
        input integer xg;
        input integer yg;
        reg [7:0] v;
        begin
            case (mode_p)
                0: pattern = mk(8'd128, 8'd128, 8'd128);
                1: begin v = (xg < GRP/2) ? 8'd0 : 8'd255; pattern = mk(v,v,v); end
                2: begin v = (yg < IMG_HEIGHT/2) ? 8'd0 : 8'd255; pattern = mk(v,v,v); end
                3: pattern = mk(8'd200, 8'd100, 8'd50);
                default: pattern = 96'd0;
            endcase
        end
    endfunction

    task reset_cnt;
        begin
            @(negedge clk);
            out_beats=0; out_user=0; out_last=0;
            white_cnt=0; black_cnt=0; oidx=0;
            wx_min=999; wx_max=-1; wy_min=999; wy_max=-1;
            samp_out = 96'hx;
            counting=1'b1;
        end
    endtask

    task feed_frame;
        input integer pat;
        integer y, x;
        begin
            in_beats = 0;
            for (y = 0; y < IMG_HEIGHT; y = y + 1)
                for (x = 0; x < GRP; x = x + 1) begin
                    @(negedge clk);
                    I_tvalid <= 1'b1;
                    I_tuser  <= (y==0) && (x==0);
                    I_tlast  <= (x==GRP-1);
                    I_tdata  <= pattern(pat, x, y);
                    in_beats = in_beats + 1;
                end
            @(negedge clk);
            I_tvalid <= 1'b0; I_tuser <= 1'b0; I_tlast <= 1'b0; I_tdata <= 96'd0;
            repeat (STAGES_WAIT) @(posedge clk);
            @(negedge clk);
            counting = 1'b0;
            if(out_beats != in_beats || out_user != 1 || out_last != IMG_HEIGHT)
                $fatal(1,"Incomplete frame in mode %0d",mode);
        end
    endtask

    task chk;
        input        cond;
        input [8*56:1] msg;
        begin
            if (cond) $display("[PASS] %0s", msg);
            else begin $display("[FAIL] %0s", msg); errors = errors + 1; end
        end
    endtask

    initial begin
        errors=0; counting=0;
        rst_n=0; mode=M_PASS;
        I_tvalid=0; I_tuser=0; I_tlast=0; I_tdata=0;
        repeat (6) @(posedge clk);
        rst_n = 1;
        repeat (4) @(posedge clk);

        //-------------------------------------------------- 直通
        mode = M_PASS;
        reset_cnt; feed_frame(3);
        $display("--- PASS 直通 --- 输入%0d拍 输出%0d拍 tuser=%0d tlast=%0d",
                 in_beats, out_beats, out_user, out_last);
        chk(out_beats == in_beats, "P1 输出拍数等于输入拍数");
        chk(out_user  == 1,        "P2 每帧恰好一个 tuser");
        chk(out_last  == IMG_HEIGHT,"P2 每行一个 tlast");
        chk(samp_out  == mk(8'd200,8'd100,8'd50), "N1 直通输出与输入一致");

        //-------------------------------------------------- 反色
        mode = M_INV;
        reset_cnt; feed_frame(3);
        $display("--- INVERT 反色 --- 采样点输出=%h", samp_out[23:0]);
        chk(samp_out[23:0] == ~{8'd200,8'd100,8'd50}, "I1 反色输出为按位取反");

        //-------------------------------------------------- 灰度
        mode = M_GRAY;
        reset_cnt; feed_frame(3);
        // Y = (77*200 + 150*100 + 29*50) >> 8 = (15400+15000+1450)>>8 = 31850>>8 = 124
        $display("--- GRAY 灰度 --- 采样点灰度=%0d (理论 124)", samp_out[7:0]);
        chk(samp_out[23:16]==samp_out[15:8] && samp_out[15:8]==samp_out[7:0],
                                              "G1 灰度三通道相同");
        chk(samp_out[7:0] == 8'd124,          "G1 灰度值符合亮度公式(无溢出)");

        //-------------------------------------------------- 灰度二值化
        mode = M_BIN;
        reset_cnt; feed_frame(0);   // 均匀128 -> 灰度128, 不大于阈值128 -> 黑
        $display("--- BIN 二值化(灰度128) --- 白=%0d 黑=%0d", white_cnt, black_cnt);
        chk(white_cnt == 0, "B1 灰度128 不超过阈值128 -> 全黑");

        mode = M_BIN;
        reset_cnt; feed_frame(3);   // (200,100,50)->灰度124 <128 -> 黑
        $display("--- BIN 二值化(灰度124) --- 白=%0d 黑=%0d", white_cnt, black_cnt);
        chk(white_cnt == 0, "B1 灰度124 低于阈值 -> 黑");

        //-------------------------------------------------- 边缘：均匀图
        mode = M_SBIN;
        reset_cnt; feed_frame(0);
        $display("--- SOBEL_BIN 均匀图 --- 白=%0d x=[%0d,%0d] y=[%0d,%0d]",
                 white_cnt, wx_min, wx_max, wy_min, wy_max);
        chk(white_cnt == 0, "S1 均匀图无虚假边缘");

        //-------------------------------------------------- 边缘：垂直
        mode = M_SBIN;
        reset_cnt; feed_frame(1);
        $display("--- SOBEL_BIN 垂直边缘 --- 白=%0d x=[%0d,%0d] y=[%0d,%0d]",
                 white_cnt, wx_min, wx_max, wy_min, wy_max);
        chk(white_cnt > 0,            "S2 垂直边缘被检出");
        chk(white_cnt < out_beats,    "S2 非全白(确是边缘不是饱和)");

        //-------------------------------------------------- 边缘：水平
        mode = M_SBIN;
        reset_cnt; feed_frame(2);
        $display("--- SOBEL_BIN 水平边缘 --- 白=%0d x=[%0d,%0d] y=[%0d,%0d]",
                 white_cnt, wx_min, wx_max, wy_min, wy_max);
        chk(white_cnt > 0, "S3 水平边缘被检出(行缓存与Gy通路正常)");

        //-------------------------------------------------- 卡通化
        mode = M_CART;
        reset_cnt; feed_frame(3);
        $display("--- CARTOON 卡通化 --- 采样点=%h (原色 c86432)", samp_out[23:0]);
        chk(samp_out[23:0] != 24'h000000, "C1 非边缘处保留色块");
        chk(samp_out[23:0] != 24'hc86432, "C1 非边缘处为量化色而非原色");

        mode = M_SMAG;
        reset_cnt; feed_frame(0);
        chk(black_cnt == out_beats, "S4 uniform image has zero Sobel magnitude");

        mode = M_HEQ;
        rst_n=0; repeat(4) @(negedge clk); rst_n=1; repeat(300) @(negedge clk);
        reset_cnt; feed_frame(3);
        chk(samp_out == {4{24'hc86432}}, "HE1 first frame is identity before a valid CDF");

        $display("========================================");
        if (errors==0) $display("[RESULT] TEST PASSED  (0 errors)");
        else           $fatal(1,"[RESULT] TEST FAILED  (%0d errors)", errors);
        $display("========================================");
        $finish;
    end

endmodule
