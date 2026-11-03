// Exposure-first AE with automatic analogue gain.
// Dark scenes first lengthen exposure up to one frame (1990 half-lines at 60 fps);
// only then is analogue gain raised, up to 12x (the 2026-09-26 indoor chart
// session chose 5x analogue x 2x digital, about 10x total).  Bright scenes first drop gain
// back to 1x, then shorten exposure.  Indoors the former exposure-only loop sat
// at 1990/2x with linear white near 4/255, which also starved AWB of samples.
// Host custom gain still owns analogue gain; AE then adjusts exposure only.
// Defaults: 2x analogue (biased LUT index 48), 1x digital (0x0080).
module ae_set #(parameter integer CLK_FREQ_HZ = 24000000)(
    input wire I_clk, I_rst,
    input wire [3:0] I_btn,
    input wire I_restore_defaults,
    input wire I_auto_enable,
    input wire [7:0] I_luma_avg,
    input wire I_luma_valid, I_ae_cfg_done, I_cam_cfg_done,
    input wire [1:0] I_manual_en,
    input wire [15:0] I_manual_ae, I_manual_ag,
    input wire I_dg_manual_en,
    input wire [15:0] I_manual_dg,
    output wire O_ae_req,
    output wire [15:0] O_ae, O_ag, O_dg
);
    localparam [15:0] AE_MIN=64, AE_MAX=1990, AE_STEP=64;
    localparam [15:0] AG_DEFAULT=48, DG_DEFAULT=16'h0080;
    // Biased gain-table indices: 16=1x, 48=2x, 80=4x, 112=8x, 128=12x.
    localparam [15:0] AG_AUTO_MIN=16, AG_AUTO_MAX=128, AG_STEP=2;
    // RAW/4 statistics include the sensor's 16-count pedestal. This target
    // precedes display gamma; the former 104..120 target overexposed gamma output.
    localparam [7:0] LUMA_LOW=60, LUMA_HIGH=72;
    // Far from target, take double steps so a dark room converges in ~2 s.
    localparam [7:0] LUMA_FAR_LOW=36, LUMA_FAR_HIGH=120;
    reg [15:0] exposure_target=AE_MAX, auto_gain=AG_DEFAULT;
    reg [15:0] exposure=AE_MAX, again=AG_DEFAULT, dgain=DG_DEFAULT;
    reg req=0, sent=0;
    reg [2:0] settling_frames=0;
    reg auto_d=0;
    wire auto_active=I_auto_enable && !I_manual_en[0];
    wire gain_auto=!I_manual_en[1];
    wire [15:0] manual_exposure=I_manual_ae<AE_MIN ? AE_MIN :
                                   I_manual_ae>AE_MAX ? AE_MAX : I_manual_ae;
    wire [15:0] gain_target=gain_auto ? auto_gain :
                              I_manual_ag<16 ? 16'd16 :
                              I_manual_ag>143 ? 16'd143 : I_manual_ag;
    wire far_dark=I_luma_avg<LUMA_FAR_LOW, far_bright=I_luma_avg>LUMA_FAR_HIGH;
    wire [15:0] exposure_step=(far_dark || far_bright) ? 2*AE_STEP : AE_STEP;
    wire [15:0] gain_step=(far_dark || far_bright) ? 2*AG_STEP : AG_STEP;
    wire [7:0] coarse=(I_manual_dg[6:5]==0) ? 8'h00 :
                      (I_manual_dg[6:5]==1) ? 8'h01 :
                      (I_manual_dg[6:5]==2) ? 8'h03 : 8'h07;
    wire [15:0] digital_target=I_dg_manual_en ?
                   {coarse, (8'h80 + {I_manual_dg[4:0],2'b00})} : DG_DEFAULT;
    assign O_ae=exposure;
    assign O_ag=again;
    assign O_dg=dgain;
    assign O_ae_req=req;

    always @(posedge I_clk or posedge I_rst) begin
        if(I_rst) begin
            exposure_target<=AE_MAX; auto_gain<=AG_DEFAULT;
            exposure<=AE_MAX; again<=AG_DEFAULT; dgain<=DG_DEFAULT;
            req<=0; sent<=0; settling_frames<=0; auto_d<=0;
        end else begin
            req<=0;
            auto_d<=auto_active;
            if(!auto_active) begin
                // Manual owns exposure, automatic integrator follows it.
                // If neither source owns it (legacy command), hold last value.
                if(I_manual_en[0]) exposure_target<=manual_exposure;
                settling_frames<=0;
            end else if(!auto_d || !I_cam_cfg_done || !I_ae_cfg_done ||
                        req || exposure!=exposure_target ||
                        again!=gain_target || dgain!=digital_target) begin
                settling_frames<=0;
            end else if(I_luma_valid) begin
                if(settling_frames==3) begin
                    settling_frames<=0;
                    if(I_luma_avg<LUMA_LOW) begin
                        if(exposure_target<AE_MAX)
                            exposure_target<=exposure_target>AE_MAX-exposure_step ?
                                             AE_MAX : exposure_target+exposure_step;
                        else if(gain_auto && auto_gain<AG_AUTO_MAX)
                            auto_gain<=auto_gain>AG_AUTO_MAX-gain_step ?
                                       AG_AUTO_MAX : auto_gain+gain_step;
                    end else if(I_luma_avg>LUMA_HIGH) begin
                        if(gain_auto && auto_gain>AG_AUTO_MIN)
                            auto_gain<=auto_gain<AG_AUTO_MIN+gain_step ?
                                       AG_AUTO_MIN : auto_gain-gain_step;
                        else
                            exposure_target<=exposure_target<AE_MIN+exposure_step ?
                                             AE_MIN : exposure_target-exposure_step;
                    end
                end else settling_frames<=settling_frames+1'b1;
            end
            // Reset control targets without interrupting an in-flight I2C group.
            if(I_restore_defaults) begin
                exposure_target<=AE_MAX;auto_gain<=AG_DEFAULT;
                settling_frames<=0;auto_d<=0;
            end
            // One coherent request; digital-only changes also trigger it.
            if(!req && I_cam_cfg_done && I_ae_cfg_done &&
               (!sent || exposure!=exposure_target ||
                again!=gain_target || dgain!=digital_target)) begin
                exposure<=exposure_target; again<=gain_target; dgain<=digital_target;
                req<=1; sent<=1; settling_frames<=0;
            end
        end
    end
endmodule

//------------------------------------------------------------------------------
// auto_exposure_stats
// 16 columns x 32 rows = 512 four-pixel samples for either camera geometry.
// Odd row stride samples both Bayer phases. 720p: x=40+80*k, y=3+23*k.
// The old stride of 256 groups sampled only the left edge of each row.
// Luma remains RAW/4 including the pedestal; existing AE thresholds are unchanged.
//------------------------------------------------------------------------------
module auto_exposure_stats #(
    parameter integer IMG_WIDTH = 1024,
    parameter integer IMG_HEIGHT = 600
) (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_frame_start,
    input  wire        I_valid,
    input  wire [39:0] I_raw10,
    output reg  [7:0]  O_luma_avg,
    output reg         O_update_toggle
);

    localparam integer GROUPS = IMG_WIDTH/4;
    localparam [8:0] SAMPLE_X_STEP = GROUPS/16;
    localparam [8:0] SAMPLE_X_FIRST = GROUPS/32;
    localparam [9:0] SAMPLE_Y_STEP = ((IMG_HEIGHT-1)/31) | 1;
    localparam [9:0] SAMPLE_Y_FIRST = (IMG_HEIGHT-1-31*SAMPLE_Y_STEP)/2;
    reg [8:0]  group_x;
    reg [8:0]  next_sample_x;
    reg [9:0]  row_y;
    reg [9:0]  next_sample_y;
    reg [9:0]  sample_cnt;
    reg [16:0] luma_sum;

    wire [11:0] raw_sum = {2'b0, I_raw10[9:0]}  +
                          {2'b0, I_raw10[19:10]} +
                          {2'b0, I_raw10[29:20]} +
                          {2'b0, I_raw10[39:30]};
    wire [7:0] sample_luma = raw_sum[11:4];

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            group_x         <= 9'd0;
            next_sample_x   <= SAMPLE_X_FIRST;
            row_y           <= 10'd0;
            next_sample_y   <= SAMPLE_Y_FIRST;
            sample_cnt      <= 10'd0;
            luma_sum        <= 17'd0;
            O_luma_avg      <= 8'd0;
            O_update_toggle <= 1'b0;
        end else if(I_frame_start) begin
            // Reject truncated/overlong frames, even if all grid points arrived.
            if(sample_cnt == 10'd512 && row_y == IMG_HEIGHT && group_x == 9'd0) begin
                O_luma_avg      <= luma_sum[16:9];
                O_update_toggle <= ~O_update_toggle;
            end
            // A frame marker may accompany the first valid group.
            group_x    <= I_valid ? 9'd1 : 9'd0;
            next_sample_x <= SAMPLE_X_FIRST;
            row_y      <= 10'd0;
            next_sample_y <= SAMPLE_Y_FIRST;
            sample_cnt <= 10'd0;
            luma_sum   <= 17'd0;
        end else if(I_valid) begin
            group_x <= (group_x == GROUPS-1) ? 9'd0 : group_x + 1'b1;
            if(group_x == GROUPS-1) begin
                next_sample_x <= SAMPLE_X_FIRST;
                row_y <= row_y + 1'b1;
                if(row_y == next_sample_y)
                    next_sample_y <= next_sample_y + SAMPLE_Y_STEP;
            end
            if((group_x == next_sample_x) && (row_y == next_sample_y) &&
               (sample_cnt < 10'd512)) begin
                next_sample_x <= next_sample_x + SAMPLE_X_STEP;
                sample_cnt <= sample_cnt + 1'b1;
                luma_sum   <= luma_sum + sample_luma;
            end
        end
    end

endmodule


	
