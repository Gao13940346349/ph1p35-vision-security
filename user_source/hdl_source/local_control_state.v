// Local keypad control only. No Ethernet command interface.
module local_control_state (
    input  wire       clk,
    input  wire       rst_n,

    // Physical press pulses, one cycle each, already debounced.
    input  wire       btn_awb,
    input  wire       btn_ae,
    input  wire       btn_cal,
    input  wire       btn_detect,
    input  wire       btn_application, // K3: toggle the security application
    input  wire       btn_mode,        // K1: step to the next display mode

    // Legacy analogue-gain inputs remain for interface compatibility only.
    // Analogue gain belongs to AE; neither input can take manual ownership.
    input wire btn_ag_up,btn_ag_down,btn_dg_up,btn_dg_down,btn_defaults,
    input wire [6:0] current_again,
    output reg restore_toggle,

    output reg         awb,
    output reg         ae,
    output reg         cal,
    output reg         detect,
    // K2 detection/tracking and K3 application pages are mutually exclusive.
    // detect remains the vision enable in BOTH pages; application selects policy.
    output reg         application,

    // Display mode, 0..7, stepped by K1.
    output reg  [2:0]  mode,

    // Exposure is a native 12-bit value; digital gain is a 7-bit step index.
    // The compatible analogue-gain outputs stay at their reset values, with
    // manual_again_en clear, so the automatic controller keeps ownership.
    output reg  [11:0] manual_exposure,
    output reg  [6:0]  manual_again,
    output reg  [6:0]  manual_dgain,
    output reg         manual_exposure_en,
    output reg         manual_again_en,
    output reg         manual_dgain_en,
    // Flips once per accepted parameter write. The manual values cross into the
    // 24 MHz camera-config domain, where a plain multi-bit synchronizer could
    // sample a torn value mid-change; the consumer captures them on this edge
    // instead, so it always takes a settled snapshot.
    output reg         param_toggle
);
    // LUT index encodes (1 + fine/32) * 2^coarse. Add 16 in Q5
    // (exactly 0.5x), then round to the nearest representable sensor value.
    function [6:0] gain_step_half;
        input [6:0] index;
        input up;
        reg [9:0] q;
        reg [1:0] stage;
        reg [9:0] fine;
        begin
            q=(10'd32+{5'd0,index[4:0]})<<index[6:5];
            if(up) q=q>488 ? 504 : q+16;
            else q=q<48 ? 32 : q-16;
            stage=q>=256 ? 3 : q>=128 ? 2 : q>=64 ? 1 : 0;
            fine=((q+(stage==0 ? 0 : (10'd1<<(stage-1))))>>stage)-32;
            if(fine>=32 && stage!=3) begin stage=stage+1'b1;fine=0;end
            if(fine>31) fine=31;
            gain_step_half={stage,fine[4:0]};
        end
    endfunction
    localparam integer MODE_COUNT = 8;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            awb    <= 1'b1;
            ae     <= 1'b1;
            cal    <= 1'b0;
            detect <= 1'b0;
            application <= 1'b0;
            mode   <= 3'd0;
            manual_exposure    <= 12'd1990;   // AE_MAX, matches the factory mode
            manual_again       <= 7'd32;      // 2.000x
            manual_dgain       <= 7'd0;       // 1.000x
            manual_exposure_en <= 1'b0;
            manual_again_en    <= 1'b0;
            manual_dgain_en    <= 1'b0;
            param_toggle       <= 1'b0;
            restore_toggle     <= 1'b0;
        end else if (btn_defaults) begin
            awb<=1;ae<=1;cal<=0;detect<=0;application<=0;mode<=0;
            manual_exposure<=1990;manual_again<=32;manual_dgain<=0;
            manual_exposure_en<=0;manual_again_en<=0;manual_dgain_en<=0;
            param_toggle<=~param_toggle;restore_toggle<=~restore_toggle;
        end else begin
            if(btn_dg_up ^ btn_dg_down) begin
                manual_dgain<=gain_step_half(manual_dgain_en ? manual_dgain : 7'd0,btn_dg_up);
                manual_dgain_en<=1;param_toggle<=~param_toggle;
            end
            // K10 is an independent AWB switch on every page.
            if (btn_awb) awb <= ~awb;
            if (btn_ae) begin
                ae <= ~ae;
                manual_exposure_en <= ae;
                param_toggle <= ~param_toggle;
            end
            // Page-key priority for simultaneous pulses:
            // defaults > K1 mode > K3 application > K2 detect > K6 details.
            if (btn_cal) begin cal<=~cal;detect<=0;application<=0;end
            if (btn_detect) begin
                cal<=0;application<=0;
                if(detect && !application) detect<=0;
                else detect<=1;
            end
            if (btn_application) begin
                cal<=0;
                if(application) begin detect<=0;application<=0;end
                else begin detect<=1;application<=1;end
            end
            if (btn_mode) begin
                mode <= (mode == MODE_COUNT - 1) ? 3'd0 : mode + 1'b1;
                detect<=0;application<=0;cal<=0;
            end
        end
    end
endmodule
