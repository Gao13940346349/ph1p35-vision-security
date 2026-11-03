//------------------------------------------------------------------------------
// Manual colour-calibration statistics.
//
// The fixed centre ROI is 512x256 pixels:
//   centred in IMG_WIDTH x IMG_HEIGHT; 720p uses x=384..895, y=232..487.
// One sample of each applicable channel is taken per four-pixel group. This
// keeps the 112.5MHz accumulator path shallow while retaining ample samples:
//   RAW Bayer: 16384 samples/channel = 2^14
//   RGB:       32768 pixels/channel = 2^15
//------------------------------------------------------------------------------

module calibration_raw_stats #(
    parameter integer IMG_WIDTH = 1024,
    parameter integer IMG_HEIGHT = 600
) (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_tuser,
    input  wire        I_tlast,
    input  wire        I_tvalid,
    input  wire [39:0] I_tdata,
    output reg  [9:0]  O_b_avg,
    output reg  [9:0]  O_gb_avg,
    output reg  [9:0]  O_gr_avg,
    output reg  [9:0]  O_r_avg,
    output reg  [9:0]  O_clip_permille,
    output reg         O_update_toggle
);

    localparam [8:0] ROI_GRP_X0 = (IMG_WIDTH-512)/8;
    localparam [8:0] ROI_GRP_X1 = ROI_GRP_X0+127;
    localparam [9:0] ROI_Y0     = (IMG_HEIGHT-256)/2;
    localparam [9:0] ROI_Y1     = ROI_Y0+255;

    reg [8:0]  S_grp_x;
    reg [9:0]  S_row_y;
    reg        S_frame_seen;
    reg [24:0] S_sum_b;
    reg [24:0] S_sum_gb;
    reg [24:0] S_sum_gr;
    reg [24:0] S_sum_r;
    reg [16:0] S_clip_count;

    wire S_in_roi = (S_grp_x >= ROI_GRP_X0) && (S_grp_x <= ROI_GRP_X1) &&
                    (S_row_y >= ROI_Y0) && (S_row_y <= ROI_Y1);
    wire [1:0] S_clip_add = {1'b0, (I_tdata[9:0] >= 10'd1000)} +
                            {1'b0, (I_tdata[19:10] >= 10'd1000)};
    // Two RAW samples/group over the ROI = 65536 samples.
    // clip_count * 1000 / 65536; 1000 = 1024 - 16 - 8.
    wire [26:0] S_clip_scaled = ({10'd0,S_clip_count} << 10) -
                                ({10'd0,S_clip_count} << 4)  -
                                ({10'd0,S_clip_count} << 3);

    // RAW stream order is P0 in [9:0] through P3 in [39:30].
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_grp_x         <= 9'd0;
            S_row_y         <= 10'd0;
            S_frame_seen    <= 1'b0;
            S_sum_b         <= 25'd0;
            S_sum_gb        <= 25'd0;
            S_sum_gr        <= 25'd0;
            S_sum_r         <= 25'd0;
            S_clip_count    <= 17'd0;
            O_b_avg         <= 10'd0;
            O_gb_avg        <= 10'd0;
            O_gr_avg        <= 10'd0;
            O_r_avg         <= 10'd0;
            O_clip_permille <= 10'd0;
            O_update_toggle <= 1'b0;
        end
        else if(I_tvalid) begin
            if(I_tuser) begin
                // Publish the completed previous frame. The first group of the
                // new frame is outside the centre ROI, so it can be discarded
                // while the accumulators and coordinates are restarted.
                if(S_frame_seen) begin
                    O_b_avg         <= S_sum_b[23:14];
                    O_gb_avg        <= S_sum_gb[23:14];
                    O_gr_avg        <= S_sum_gr[23:14];
                    O_r_avg         <= S_sum_r[23:14];
                    O_clip_permille <= S_clip_scaled[25:16];
                    O_update_toggle <= ~O_update_toggle;
                end
                S_frame_seen <= 1'b1;
                S_sum_b      <= 25'd0;
                S_sum_gb     <= 25'd0;
                S_sum_gr     <= 25'd0;
                S_sum_r      <= 25'd0;
                S_clip_count <= 17'd0;
                S_grp_x      <= 9'd1;
                S_row_y      <= 10'd0;
            end
            else begin
                if(S_in_roi) begin
                    S_clip_count <= S_clip_count + S_clip_add;
                    if(!S_row_y[0]) begin       // BGGR even row: B Gb B Gb
                        S_sum_b  <= S_sum_b  + I_tdata[9:0];
                        S_sum_gb <= S_sum_gb + I_tdata[19:10];
                    end
                    else begin                  // BGGR odd row: Gr R Gr R
                        S_sum_gr <= S_sum_gr + I_tdata[9:0];
                        S_sum_r  <= S_sum_r  + I_tdata[19:10];
                    end
                end

                if(I_tlast) begin
                    S_grp_x <= 9'd0;
                    S_row_y <= S_row_y + 10'd1;
                end
                else begin
                    S_grp_x <= S_grp_x + 9'd1;
                end
            end
        end
    end

endmodule


module calibration_rgb_stats #(
    parameter integer IMG_WIDTH = 1024,
    parameter integer IMG_HEIGHT = 600
) (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_tuser,
    input  wire        I_tlast,
    input  wire        I_tvalid,
    input  wire [95:0] I_tdata,
    output reg  [7:0]  O_r_avg,
    output reg  [7:0]  O_g_avg,
    output reg  [7:0]  O_b_avg,
    output reg  [9:0]  O_clip_permille,
    output reg         O_update_toggle
);

    localparam [8:0] ROI_GRP_X0 = (IMG_WIDTH-512)/8;
    localparam [8:0] ROI_GRP_X1 = ROI_GRP_X0+127;
    localparam [9:0] ROI_Y0     = (IMG_HEIGHT-256)/2;
    localparam [9:0] ROI_Y1     = ROI_Y0+255;

    reg [8:0]  S_grp_x;
    reg [9:0]  S_row_y;
    reg        S_frame_seen;
    reg [24:0] S_sum_r;
    reg [24:0] S_sum_g;
    reg [24:0] S_sum_b;
    reg [17:0] S_clip_count;

    wire S_in_roi = (S_grp_x >= ROI_GRP_X0) && (S_grp_x <= ROI_GRP_X1) &&
                    (S_row_y >= ROI_Y0) && (S_row_y <= ROI_Y1);

    wire [7:0] S_r0 = I_tdata[95:88];
    wire [7:0] S_g0 = I_tdata[87:80];
    wire [7:0] S_b0 = I_tdata[79:72];
    wire S_clip0 = (S_r0 >= 8'd250) || (S_g0 >= 8'd250) || (S_b0 >= 8'd250);

    // clip_count * 1000 / 32768. 1000 = 1024 - 16 - 8.
    wire [27:0] S_clip_scaled = ({10'd0,S_clip_count} << 10) -
                                ({10'd0,S_clip_count} << 4)  -
                                ({10'd0,S_clip_count} << 3);

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_grp_x         <= 9'd0;
            S_row_y         <= 10'd0;
            S_frame_seen    <= 1'b0;
            S_sum_r         <= 25'd0;
            S_sum_g         <= 25'd0;
            S_sum_b         <= 25'd0;
            S_clip_count    <= 18'd0;
            O_r_avg         <= 8'd0;
            O_g_avg         <= 8'd0;
            O_b_avg         <= 8'd0;
            O_clip_permille <= 10'd0;
            O_update_toggle <= 1'b0;
        end
        else if(I_tvalid) begin
            if(I_tuser) begin
                if(S_frame_seen) begin
                    O_r_avg         <= S_sum_r[22:15];
                    O_g_avg         <= S_sum_g[22:15];
                    O_b_avg         <= S_sum_b[22:15];
                    O_clip_permille <= S_clip_scaled[24:15];
                    O_update_toggle <= ~O_update_toggle;
                end
                S_frame_seen <= 1'b1;
                S_sum_r      <= 25'd0;
                S_sum_g      <= 25'd0;
                S_sum_b      <= 25'd0;
                S_clip_count <= 18'd0;
                S_grp_x      <= 9'd1;
                S_row_y      <= 10'd0;
            end
            else begin
                if(S_in_roi) begin
                    S_sum_r      <= S_sum_r + S_r0;
                    S_sum_g      <= S_sum_g + S_g0;
                    S_sum_b      <= S_sum_b + S_b0;
                    S_clip_count <= S_clip_count + S_clip0;
                end

                if(I_tlast) begin
                    S_grp_x <= 9'd0;
                    S_row_y <= S_row_y + 10'd1;
                end
                else begin
                    S_grp_x <= S_grp_x + 9'd1;
                end
            end
        end
    end

endmodule


//------------------------------------------------------------------------------
// One shared, sequential binary-to-BCD converter. It formats all eleven stable
// calibration values in 176 pixel-clock cycles, once per HDMI frame.
//------------------------------------------------------------------------------
module calibration_bcd_formatter (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_start,
    input  wire [9:0]  I_raw_b,
    input  wire [9:0]  I_raw_gb,
    input  wire [9:0]  I_raw_gr,
    input  wire [9:0]  I_raw_r,
    input  wire [7:0]  I_rgb_r,
    input  wire [7:0]  I_rgb_g,
    input  wire [7:0]  I_rgb_b,
    input  wire [9:0]  I_clip_permille,
    input  wire [15:0] I_exposure,
    input  wire [15:0] I_gain,
    input wire [15:0] I_dgain,
    output reg  [19:0] O_raw_b_bcd,
    output reg  [19:0] O_raw_gb_bcd,
    output reg  [19:0] O_raw_gr_bcd,
    output reg  [19:0] O_raw_r_bcd,
    output reg  [19:0] O_rgb_r_bcd,
    output reg  [19:0] O_rgb_g_bcd,
    output reg  [19:0] O_rgb_b_bcd,
    output reg  [19:0] O_clip_bcd,
    output reg  [19:0] O_exposure_bcd,
    output reg  [19:0] O_gain_bcd,
    output reg [19:0] O_dgain_bcd
);

    reg        S_busy;
    reg [3:0]  S_value_idx;
    reg [4:0]  S_bit_idx;
    reg [15:0] S_bin;
    reg [19:0] S_bcd;

    reg [15:0] S_snap_raw_b;
    reg [15:0] S_snap_raw_gb;
    reg [15:0] S_snap_raw_gr;
    reg [15:0] S_snap_raw_r;
    reg [15:0] S_snap_rgb_r;
    reg [15:0] S_snap_rgb_g;
    reg [15:0] S_snap_rgb_b;
    reg [15:0] S_snap_clip;
    reg [15:0] S_snap_exposure;
    reg [15:0] S_snap_gain;
    reg [15:0] S_snap_dgain;

    reg [19:0] S_bcd_add3;
    reg [35:0] S_shifted;
    integer i;

    always @(*) begin
        S_bcd_add3 = S_bcd;
        for(i=0; i<5; i=i+1) begin
            if(S_bcd_add3[i*4 +: 4] >= 4'd5)
                S_bcd_add3[i*4 +: 4] = S_bcd_add3[i*4 +: 4] + 4'd3;
        end
        S_shifted = {S_bcd_add3, S_bin} << 1;
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_busy          <= 1'b0;
            S_value_idx     <= 4'd0;
            S_bit_idx       <= 5'd0;
            S_bin           <= 16'd0;
            S_bcd           <= 20'd0;
            S_snap_raw_b    <= 16'd0;
            S_snap_raw_gb   <= 16'd0;
            S_snap_raw_gr   <= 16'd0;
            S_snap_raw_r    <= 16'd0;
            S_snap_rgb_r    <= 16'd0;
            S_snap_rgb_g    <= 16'd0;
            S_snap_rgb_b    <= 16'd0;
            S_snap_clip     <= 16'd0;
            S_snap_exposure <= 16'd0;
            S_snap_gain     <= 16'd0;
            S_snap_dgain <= 0;O_dgain_bcd<=0;
            O_raw_b_bcd     <= 20'd0;
            O_raw_gb_bcd    <= 20'd0;
            O_raw_gr_bcd    <= 20'd0;
            O_raw_r_bcd     <= 20'd0;
            O_rgb_r_bcd     <= 20'd0;
            O_rgb_g_bcd     <= 20'd0;
            O_rgb_b_bcd     <= 20'd0;
            O_clip_bcd      <= 20'd0;
            O_exposure_bcd  <= 20'd0;
            O_gain_bcd      <= 20'd0;
        end
        else if(!S_busy) begin
            if(I_start) begin
                S_snap_raw_b    <= {6'd0,I_raw_b};
                S_snap_raw_gb   <= {6'd0,I_raw_gb};
                S_snap_raw_gr   <= {6'd0,I_raw_gr};
                S_snap_raw_r    <= {6'd0,I_raw_r};
                S_snap_rgb_r    <= {8'd0,I_rgb_r};
                S_snap_rgb_g    <= {8'd0,I_rgb_g};
                S_snap_rgb_b    <= {8'd0,I_rgb_b};
                S_snap_clip     <= {6'd0,I_clip_permille};
                S_snap_exposure <= I_exposure;
                S_snap_gain     <= I_gain;
                S_snap_dgain <= I_dgain;
                S_bin           <= {6'd0,I_raw_b};
                S_bcd           <= 20'd0;
                S_value_idx     <= 4'd0;
                S_bit_idx       <= 5'd0;
                S_busy          <= 1'b1;
            end
        end
        else begin
            if(S_bit_idx == 5'd15) begin
                case(S_value_idx)
                    4'd0: O_raw_b_bcd    <= S_shifted[35:16];
                    4'd1: O_raw_gb_bcd   <= S_shifted[35:16];
                    4'd2: O_raw_gr_bcd   <= S_shifted[35:16];
                    4'd3: O_raw_r_bcd    <= S_shifted[35:16];
                    4'd4: O_rgb_r_bcd    <= S_shifted[35:16];
                    4'd5: O_rgb_g_bcd    <= S_shifted[35:16];
                    4'd6: O_rgb_b_bcd    <= S_shifted[35:16];
                    4'd7: O_clip_bcd     <= S_shifted[35:16];
                    4'd8: O_exposure_bcd <= S_shifted[35:16];
                    4'd9: O_gain_bcd     <= S_shifted[35:16];
                    4'd10: O_dgain_bcd <= S_shifted[35:16];
                    default: ;
                endcase

                if(S_value_idx == 4'd10) begin
                    S_busy <= 1'b0;
                end
                else begin
                    S_value_idx <= S_value_idx + 4'd1;
                    S_bit_idx   <= 5'd0;
                    S_bcd       <= 20'd0;
                    case(S_value_idx + 4'd1)
                        4'd1: S_bin <= S_snap_raw_gb;
                        4'd2: S_bin <= S_snap_raw_gr;
                        4'd3: S_bin <= S_snap_raw_r;
                        4'd4: S_bin <= S_snap_rgb_r;
                        4'd5: S_bin <= S_snap_rgb_g;
                        4'd6: S_bin <= S_snap_rgb_b;
                        4'd7: S_bin <= S_snap_clip;
                        4'd8: S_bin <= S_snap_exposure;
                        4'd9: S_bin <= S_snap_gain;
                        4'd10: S_bin <= S_snap_dgain;
                        default: S_bin <= 16'd0;
                    endcase
                end
            end
            else begin
                S_bin     <= S_shifted[15:0];
                S_bcd     <= S_shifted[35:16];
                S_bit_idx <= S_bit_idx + 5'd1;
            end
        end
    end

endmodule
