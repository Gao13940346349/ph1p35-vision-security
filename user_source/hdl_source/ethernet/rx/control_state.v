// Single state layer for the five host/button control states.
//
// Physical buttons and host commands are two event sources that must converge
// here rather than each driving their own copy, which is what
// CONTROL_ASSIGNMENT_20260907.md requires. The merge rule is "last valid event
// wins":
//
//   - A host command carries an absolute SET of all four flags, so it is
//     written straight through. That makes it idempotent, which matters because
//     the host sends each command three times to survive a lost datagram.
//   - A button press toggles only its own flag.
//
// An XOR-style merge (hold ^ event) was rejected: with the host's 3x repeat the
// same datagram would flip the state three times and end up wrong.
//
// cmd_toggle is a level that flips once per accepted datagram, so it can be
// synchronized into this slower domain. cmd_bus is registered in the same
// cycle the flip happens and is not written again until the next datagram, so
// by the time the third synchronizer stage has seen the edge the bus has been
// stable for far longer than the synchronizer needs. The capture and the
// apply are deliberately on different cycles so the merge reads the bus that
// belongs to this edge.
module control_state (
    input  wire       clk,
    input  wire       rst_n,

    // Physical press pulses, one cycle each, already debounced.
    input  wire       btn_awb,
    input  wire       btn_ae,
    input  wire       btn_cal,
    input  wire       btn_detect,
    input  wire       btn_mode,        // K1: step to the next display mode

    input wire btn_ag_up,btn_ag_down,btn_dg_up,btn_dg_down,btn_defaults,
    input wire [6:0] current_again,
    output reg restore_toggle,

    // Host command, cmd_toggle in the clk domain. Bus layout:
    //   [2:0]  mode
    //   [3]    awb          [4] ae        [5] cal       [6] detect
    //   [8:7]  param_sel    0=exposure 1=analogue gain 2=digital gain
    //   [9]    manual_en    write param_sel's value and hand it to manual
    //   [17:10] param_val   the 8-bit target
    input  wire [17:0] cmd_bus,
    input  wire        cmd_toggle,

    output reg         awb,
    output reg         ae,
    output reg         cal,
    output reg         detect,

    // Display mode, 0..7. Owned here so the host can set it absolutely and a
    // button press can still step it.
    output reg  [2:0]  mode,

    // Manual image controls driven from the host. Exposure and analogue gain
    // are 12/7-bit values in their native units, digital gain is the 7-bit
    // step index. Each has a manual-enable bit; when clear the corresponding
    // automatic controller keeps ownership.
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
    reg [2:0]  toggle_sync;
    reg        toggle_seen;
    reg [17:0] bus_hold;
    reg        stb_r;

    wire stb = toggle_sync[2] ^ toggle_seen;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            toggle_sync <= 3'b000;
            toggle_seen <= 1'b0;
            bus_hold    <= 18'd0;
            stb_r       <= 1'b0;
        end else begin
            toggle_sync <= {toggle_sync[1:0], cmd_toggle};
            stb_r       <= stb;
            if (stb) begin
                bus_hold    <= cmd_bus;
                toggle_seen <= toggle_sync[2];
            end
        end
    end

    // Parameter unpacking. Only the selected parameter is touched, so dragging
    // one slider never disturbs another. The 8-bit target is widened per
    // parameter to its natural range; exposure is the only 12-bit one.
    // Exposure is the only 12-bit parameter; the host sends its 8-bit knob and
    // this maps it to the register range 64..2040 in steps of 8. The high end
    // is deliberately kept below ae_set's own clamp of 1990, which lives at the
    // 2*VTS-10 limit for the factory mode.
    wire [11:0] exp_target = 12'd64 + (bus_hold[17:10] * 12'd8);
    wire [6:0]  gain_target = bus_hold[16:10];         // 0..127 index

    // The wire format carries the mode as an absolute 3-bit value in the same
    // byte as the flags, so a dropdown jump of several positions is just one
    // write. Only the flags need the careful merge reasoning above; the mode
    // is written the same way and stepped by K1 in between.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            awb    <= 1'b1;
            ae     <= 1'b1;
            cal    <= 1'b0;
            detect <= 1'b0;
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
            awb<=1;ae<=1;cal<=0;detect<=0;mode<=0;
            manual_exposure<=1990;manual_again<=32;manual_dgain<=0;
            manual_exposure_en<=0;manual_again_en<=0;manual_dgain_en<=0;
            param_toggle<=~param_toggle;restore_toggle<=~restore_toggle;
        end else if (stb_r) begin
            awb    <= bus_hold[3];
            ae     <= bus_hold[4];
            cal    <= bus_hold[5];
            detect <= bus_hold[6];
            mode   <= bus_hold[2:0];

            // 只有选中某个参数时才翻转 param_toggle：上层用于捕获一次
            // 稳定快照，所以"更新一次翻转一次"正好。
            param_toggle <= ~param_toggle;
            manual_exposure_en <= !bus_hold[4];
            case (bus_hold[8:7])
                2'd0: begin
                    manual_exposure    <= exp_target;
                    manual_exposure_en <= bus_hold[9];
                    ae                 <= !bus_hold[9];
                end
                2'd1: begin
                    manual_again       <= gain_target;
                    manual_again_en    <= bus_hold[9];
                end
                2'd2: begin
                    manual_dgain       <= gain_target;
                    manual_dgain_en    <= bus_hold[9];
                end
                default: begin end // State-only command.
            endcase
        end else begin
            if(btn_ag_up ^ btn_ag_down) begin
                manual_again<=gain_step_half(manual_again_en ? manual_again : current_again,btn_ag_up);
                manual_again_en<=1;param_toggle<=~param_toggle;
            end
            if(btn_dg_up ^ btn_dg_down) begin
                manual_dgain<=gain_step_half(manual_dgain_en ? manual_dgain : 7'd0,btn_dg_up);
                manual_dgain_en<=1;param_toggle<=~param_toggle;
            end
            if (btn_awb)    awb    <= ~awb;
            if (btn_ae) begin
                ae <= ~ae;
                manual_exposure_en <= ae;
                param_toggle <= ~param_toggle;
            end
            if (btn_cal) begin cal<=~cal;if(!cal) detect<=0;end
            if (btn_detect) begin detect<=~detect;if(!detect) cal<=0;end
            if (btn_mode) begin
                mode <= (mode == MODE_COUNT - 1) ? 3'd0 : mode + 1'b1;
                detect<=0;cal<=0;
            end
        end
    end
endmodule
