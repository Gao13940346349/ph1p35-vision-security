module edge_detect_96 #(
    parameter IMG_WIDTH  = 1024,
    parameter IMG_HEIGHT = 600,
    parameter EDGE_TH    = 11'd132
)(
    input               I_clk,
    input               I_rst_n,
    input               I_tlast,
    input               I_tuser,
    input      [95:0]   I_tdata,
    input               I_tvalid,
    input               I_tready,
    output reg          O_tlast,
    output reg          O_tuser,
    output reg [95:0]   O_tdata,
    output reg          O_tvalid,
    output              O_tready
);

assign O_tready = I_tready;

wire [23:0] pix0 = I_tdata[95:72];
wire [23:0] pix1 = I_tdata[71:48];
wire [23:0] pix2 = I_tdata[47:24];
wire [23:0] pix3 = I_tdata[23:0];

// [修正-位宽溢出] 原实现为:
//     wire [7:0] gray0 = (pix0[23:16] + pix0[15:8] + pix0[7:0]) / 8'd3;
// 三个 8bit 分量相加最大为 765, 但表达式位宽被 8bit 的 wire 限制, 结果在 8bit 内回绕。
// 实测: (86,86,86)->0, (128,128,128)->42, (255,255,255)->84, 且非单调(85->85 而 86->0)。
// 正常图像绝大多数像素 R+G+B>255, 灰度基本失真, 导致梯度幅值被压低到检测不出真实边缘。
// 修正方法: 用 10bit 中间量承载求和结果, 再做除法。
wire [9:0] rgb_sum0 = pix0[23:16] + pix0[15:8] + pix0[7:0];
wire [9:0] rgb_sum1 = pix1[23:16] + pix1[15:8] + pix1[7:0];
wire [9:0] rgb_sum2 = pix2[23:16] + pix2[15:8] + pix2[7:0];
wire [9:0] rgb_sum3 = pix3[23:16] + pix3[15:8] + pix3[7:0];

wire [7:0] gray0 = rgb_sum0 / 10'd3;
wire [7:0] gray1 = rgb_sum1 / 10'd3;
wire [7:0] gray2 = rgb_sum2 / 10'd3;
wire [7:0] gray3 = rgb_sum3 / 10'd3;

localparam IMG_WIDTH_GRP = (IMG_WIDTH >> 2);

(* ram_style = "block" *) reg [7:0] line1_mem [0:IMG_WIDTH_GRP-1];
(* ram_style = "block" *) reg [7:0] line2_mem [0:IMG_WIDTH_GRP-1];

reg [10:0] x_grp;
reg [10:0] y_pos;

reg [7:0] r0c0, r0c1, r0c2;
reg [7:0] r1c0, r1c1, r1c2;
reg [7:0] r2c0, r2c1, r2c2;

reg [7:0] grp_gray;
reg [9:0] grp_gray_sum;      // [修正-位宽溢出] 承载 4 个灰度之和(最大1020), 8bit 装不下
reg [7:0] prev_grp_gray_d1;
reg [7:0] prev_grp_gray_d2;
reg [7:0] smooth_gray;
reg [7:0] top_v;
reg [7:0] mid_v;

reg signed [12:0] gx;
reg signed [12:0] gy;
reg [12:0] abs_gx;
reg [12:0] abs_gy;
reg [13:0] mag;
reg        edge_flag;
reg        edge_hist_d1;
reg        edge_hist_d2;
reg        edge_flag_filt;

reg [23:0] out_pix;

integer i;

always @(posedge I_clk or negedge I_rst_n) begin
    if(!I_rst_n) begin
        O_tlast  <= 1'b0;
        O_tuser  <= 1'b0;
        O_tdata  <= 96'd0;
        O_tvalid <= 1'b0;
        x_grp <= 11'd0;
        y_pos <= 11'd0;
        edge_hist_d1 = 1'b0;
        edge_hist_d2 = 1'b0;
        prev_grp_gray_d1 = 8'd0;
        prev_grp_gray_d2 = 8'd0;
        smooth_gray = 8'd0;
        r0c0 = 8'd0; r0c1 = 8'd0; r0c2 = 8'd0;
        r1c0 = 8'd0; r1c1 = 8'd0; r1c2 = 8'd0;
        r2c0 = 8'd0; r2c1 = 8'd0; r2c2 = 8'd0;
        for(i=0; i<IMG_WIDTH_GRP; i=i+1) begin
            line1_mem[i] <= 8'd0;
            line2_mem[i] <= 8'd0;
        end
    end
    else begin
        O_tlast  <= I_tlast;
        O_tuser  <= I_tuser;
        O_tvalid <= I_tvalid;

        if(I_tvalid) begin
            if(I_tuser) begin
                x_grp <= 11'd0;
                y_pos <= 11'd0;
                edge_hist_d1 = 1'b0;
                edge_hist_d2 = 1'b0;
                prev_grp_gray_d1 = 8'd0;
                prev_grp_gray_d2 = 8'd0;
                r0c0 = 8'd0; r0c1 = 8'd0; r0c2 = 8'd0;
                r1c0 = 8'd0; r1c1 = 8'd0; r1c2 = 8'd0;
                r2c0 = 8'd0; r2c1 = 8'd0; r2c2 = 8'd0;
            end

            // [修正-位宽溢出] 原为 grp_gray = (gray0+gray1+gray2+gray3) >> 2;
            // 4 个 8bit 灰度相加最大 1020, 表达式位宽受 8bit 的 grp_gray 限制而回绕,
            // 白色(255)会被算成 63, 动态范围损失 4 倍, 梯度幅值达不到阈值。
            grp_gray_sum = gray0 + gray1 + gray2 + gray3;
            grp_gray     = grp_gray_sum >> 2;

            if(x_grp == 11'd0)
                smooth_gray = grp_gray;
            else if(x_grp == 11'd1)
                // [修正-位宽溢出] 原为 (grp_gray + prev_grp_gray_d1) >> 1;
                // 两个 8bit 相加最大 510, 同样会在 8bit 内回绕。
                smooth_gray = ({2'b00, grp_gray} + {2'b00, prev_grp_gray_d1}) >> 1;
            else
                smooth_gray = ({2'b00, grp_gray} + ({2'b00, prev_grp_gray_d1} << 1) + {2'b00, prev_grp_gray_d2}) >> 2;
            prev_grp_gray_d2 = prev_grp_gray_d1;
            prev_grp_gray_d1 = grp_gray;

            top_v = line2_mem[x_grp];
            mid_v = line1_mem[x_grp];

            r0c0 = r0c1; r0c1 = r0c2; r0c2 = top_v;
            r1c0 = r1c1; r1c1 = r1c2; r1c2 = mid_v;
            r2c0 = r2c1; r2c1 = r2c2; r2c2 = smooth_gray;

            if((x_grp >= 11'd2) && (y_pos >= 11'd2)) begin
                gx = -$signed({1'b0,r0c0}) + $signed({1'b0,r0c2})
                   - ($signed({1'b0,r1c0}) <<< 1) + ($signed({1'b0,r1c2}) <<< 1)
                   - $signed({1'b0,r2c0}) + $signed({1'b0,r2c2});

                gy =  $signed({1'b0,r0c0}) + ($signed({1'b0,r0c1}) <<< 1) + $signed({1'b0,r0c2})
                   - $signed({1'b0,r2c0}) - ($signed({1'b0,r2c1}) <<< 1) - $signed({1'b0,r2c2});

                abs_gx = gx[12] ? (~gx + 1'b1) : gx;
                abs_gy = gy[12] ? (~gy + 1'b1) : gy;
                mag    = abs_gx + abs_gy;
                edge_flag = (mag >= EDGE_TH);
            end
            else begin
                edge_flag = 1'b0;
            end

            // 3-point majority filter: suppress isolated single-point edge noise.
            edge_flag_filt = (edge_flag & edge_hist_d1) |
                             (edge_flag & edge_hist_d2) |
                             (edge_hist_d1 & edge_hist_d2);
            edge_hist_d2 = edge_hist_d1;
            edge_hist_d1 = edge_flag;

            out_pix = edge_flag_filt ? 24'hFFFFFF : 24'h000000;
            O_tdata <= {out_pix, out_pix, out_pix, out_pix};

            line2_mem[x_grp] <= mid_v;
            line1_mem[x_grp] <= smooth_gray;

            if(x_grp == IMG_WIDTH_GRP - 1) begin
                x_grp <= 11'd0;
                if(y_pos == IMG_HEIGHT - 1)
                    y_pos <= 11'd0;
                else
                    y_pos <= y_pos + 1'b1;

                edge_hist_d1 = 1'b0;
                edge_hist_d2 = 1'b0;

                r0c0 = 8'd0; r0c1 = 8'd0; r0c2 = 8'd0;
                r1c0 = 8'd0; r1c1 = 8'd0; r1c2 = 8'd0;
                r2c0 = 8'd0; r2c1 = 8'd0; r2c2 = 8'd0;
            end
            else begin
                x_grp <= x_grp + 1'b1;
            end
        end
        else begin
            O_tdata <= 96'd0;
        end
    end
end

endmodule
