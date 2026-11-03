// Display tone curve applied to the four-pixel RGB888 ISP stream.
//
// Why: the SC500CS outputs linear RAW and the ISP chain has no gamma, so the
// picture is displayed linearly and reads as dark and washed out. The sensor
// itself has no gamma or contrast register, so the correction has to live here.
// Presets 0..4 use the same curve on all channels.  Preset 5 uses three
// measured, monotonic channel curves.  It folds brightness and neutral-axis
// correction into the existing lookup resources without a multiplier matrix.
//
// Placement: instantiated immediately before video_proc_96 in isp_top.v, which
// is the last point that both output taps share. HDMI takes the result through
// video_proc_96 -> 96to128, UDP takes it through video_proc_96 directly. Doing
// it here means one curve serves both, and it sits before the edge/shape
// algorithms so their thresholds keep their existing meaning.
//
// Timing: this runs on MIPI_RX_BYTE_CLK at 112.5 MHz (8.888 ns). The median
// and lookup have separate register boundaries. Total latency is three cycles.
//
// Data layout (matches video_proc_96.v): I_tdata[95:72] is the leftmost pixel,
// [23:0] the rightmost, three bytes per pixel in R,G,B order.
module luma_curve_96 #(
    // 0=1.8  1=2.0  2=2.2  3=2.4  4=linear  5=chart calibrated RGB
    parameter integer CURVE_SEL = 2,
    // Constant bypass, for A/B comparing on hardware without editing the
    // curve. Folds away when constant.
    parameter integer CURVE_ENABLE = 1,
    parameter integer DENOISE_ENABLE = 1
) (
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
    wire [95:0] lut_out;
    wire [95:0] denoised_beat;
    reg  [95:0] held_beat, filtered_beat;
    reg  [23:0] left_pixel;
    reg held_last, held_user, held_valid, left_valid;
    reg filtered_last, filtered_user, filtered_valid;

    // A three-pixel median removes isolated sensor speckles without changing
    // a broad edge. It is limited to shadows before the steep display curve.
    // Hold one four-pixel beat so lane 3 can see lane 0 of the next beat.
    function [7:0] median3;
        input [7:0] a, b, c;
        begin
            if (a > b) begin
                if (b > c) median3 = b;
                else if (a > c) median3 = c;
                else median3 = a;
            end else begin
                if (a > c) median3 = a;
                else if (b > c) median3 = c;
                else median3 = b;
            end
        end
    endfunction

    // Twelve lookups per clock: four pixels x three channels. Each distributed
    // RAM has a single read port, so each needs its own copy of the curve.
    genvar p, c;
    generate
        for (p = 0; p < 4; p = p + 1) begin : g_pixel
            for (c = 0; c < 3; c = c + 1) begin : g_chan
                wire [7:0] center, left, right;
                assign center = held_beat[95 - 24*p - 8*c -: 8];
                if (p == 0) begin : g_first
                    assign left = (left_valid && !held_user) ?
                                  left_pixel[23 - 8*c -: 8] : center;
                end else begin : g_other_left
                    assign left = held_beat[95 - 24*(p-1) - 8*c -: 8];
                end
                if (p == 3) begin : g_last
                    assign right = (I_tvalid && !I_tuser && !held_last) ?
                                   I_tdata[95 - 8*c -: 8] : center;
                end else begin : g_other_right
                    assign right = held_beat[95 - 24*(p+1) - 8*c -: 8];
                end
                assign denoised_beat[95 - 24*p - 8*c -: 8] =
                    (DENOISE_ENABLE && center < 8'd40 &&
                     left < 8'd40 && right < 8'd40) ?
                    median3(left, center, right) : center;
                // Component base: leftmost pixel starts at bit 95, and each
                // pixel steps down 24 bits then each channel 8 bits.
                curve_lut256 #(
                    .CURVE_SEL  (CURVE_SEL),
                    .CHANNEL_SEL(c)
                ) u_lut (
                    .addr (filtered_beat[95 - 24*p - 8*c -: 8]),
                    .dout (lut_out[95 - 24*p - 8*c -: 8])
                );
            end
        end
    endgenerate

    // Hold, median, then lookup. Delay every sideband by the same three clocks.
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            held_beat <= 96'd0;
            held_last <= 1'b0;
            held_user <= 1'b0;
            held_valid <= 1'b0;
            left_pixel <= 24'd0;
            left_valid <= 1'b0;
            filtered_beat <= 96'd0;
            filtered_last <= 1'b0;
            filtered_user <= 1'b0;
            filtered_valid <= 1'b0;
            O_tlast  <= 1'b0;
            O_tuser  <= 1'b0;
            O_tvalid <= 1'b0;
            O_tdata  <= 96'd0;
        end else begin
            held_beat <= I_tdata;
            held_last <= I_tlast;
            held_user <= I_tuser;
            held_valid <= I_tvalid;
            left_pixel <= held_beat[23:0];
            left_valid <= held_valid && !held_last;
            filtered_beat <= denoised_beat;
            filtered_last <= held_last;
            filtered_user <= held_user;
            filtered_valid <= held_valid;
            O_tlast  <= filtered_last;
            O_tuser  <= filtered_user;
            O_tvalid <= filtered_valid;
            O_tdata  <= CURVE_ENABLE ? lut_out : filtered_beat;
        end
    end
endmodule
