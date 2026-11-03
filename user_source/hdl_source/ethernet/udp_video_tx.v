// DDR-backed UDP transmitter: raw RGB888 720p30 by default.
//
// The camera/ISP already writes complete 1280x720 RGB888 frames into a
// four-slot DDR ring. Select every second 60 Hz frame, fetch short DDR bursts,
// and cross through a 512-word FIFO. Once a frame has started, neither a
// temporary arbitration stall nor the next camera frame may abandon it.
module udp_video_tx #(
    parameter integer COMPRESS_ENABLE = 0,
    // Factory 720p source is 60 fps. Raw RGB888 defaults to 60/2=30 fps;
    // optional B8 compression defaults to 60/3=20 fps.
    parameter integer FRAME_DIV = COMPRESS_ENABLE ? 3 : 2,
    // A pending Ethernet request has not issued any DDR command yet, so it is
    // always safe to abandon it.  The watchdog prevents a stopped/continuous
    // video client from holding both established clients paused forever.
    parameter integer PENDING_TIMEOUT_CYCLES = 16383,
    // Once a burst has issued commands, its responses cannot safely be handed
    // to another client. The watchdog is diagnostic only: the burst remains
    // owner and resumes when the controller makes progress.
    parameter integer BURST_TIMEOUT_CYCLES = 65535
) (
    input  wire         sys_clk_50m,
    input  wire         ddr_clk,
    input  wire         ddr_rst_n,
    input  wire         ddr_frame_start,
    input  wire [1:0]   ddr_frame_slot,
    input  wire         ddr_video_wr_busy,
    input  wire         ddr_video_rd_busy,
    input  wire         ddr_video_rd_idle,
    output wire         ddr_reserve,
    output wire         ddr_response_owner,
    output wire         ddr_rd_en,
    output wire [24:0]  ddr_rd_addr,
    input  wire         ddr_user_ready,
    input  wire         ddr_user_rd_valid,
    input  wire [127:0] ddr_user_rd_data,
    // Control state from the 100 MHz button/switch layer. Synchronized into
    // clk125 below and echoed in the packet header so the host can mirror the
    // physical buttons.
    input  wire [2:0]   ctrl_mode,
    input  wire         ctrl_awb,
    input  wire         ctrl_ae,
    input  wire         ctrl_cal,
    input  wire         ctrl_detect,
    input  wire         ctrl_manual_exposure,
    input  wire         ctrl_manual_again,
    input  wire         ctrl_manual_dgain,
    input  wire         camera_param_clk,
    input  wire [63:0]  camera_applied_params,
    output wire [3:0]   debug_status,
    output wire         phy_reset_n,
    output wire         rgmii_txc,
    output wire [3:0]   rgmii_txd,
    output wire         rgmii_tx_ctl,
    // Ethernet clock domain, shared with the receive logic so the design needs
    // only one 125 MHz PLL. The reset is aligned to PLL lock and PHY release.
    output wire         eth_clk125,
    output wire         eth_rst_n
);
    localparam integer FRAME_WORDS = 172800; // 1280*720*3/16
    localparam integer BURST_WORDS = 64;
    localparam [24:0] IMAGE_BASE_ADDR_0 = 25'd0;
    localparam [24:0] IMAGE_BASE_ADDR_1 = 25'd7000000;
    localparam [24:0] IMAGE_BASE_ADDR_2 = 25'd14000000;
    localparam [24:0] IMAGE_BASE_ADDR_3 = 25'd21000000;

    wire clk125;
    // Bundled-data mailbox. Source holds the complete ACKed snapshot until
    // the destination acknowledges it; no independently sampled gain bits.
    reg [63:0] param_hold = 0;
    reg param_req = 0, param_ack = 0;
    reg [2:0] param_ack_sync = 0, param_req_sync = 0;
    reg [63:0] params_eth = 0;
    always @(posedge camera_param_clk or negedge ddr_rst_n) begin
      if(!ddr_rst_n) begin param_ack_sync<=0;param_req<=0;param_hold<=0;end
      else begin
        param_ack_sync <= {param_ack_sync[1:0], param_ack};
        if (param_ack_sync[2] == param_req && param_hold != camera_applied_params) begin
            param_hold <= camera_applied_params;
            param_req <= ~param_req;
        end
      end
    end
    always @(posedge clk125 or negedge ddr_rst_n) begin
      if(!ddr_rst_n) begin param_req_sync<=0;param_ack<=0;params_eth<=0;end
      else begin
        param_req_sync <= {param_req_sync[1:0], param_req};
        if (param_req_sync[2] != param_ack) begin
            params_eth <= param_hold;
            param_ack <= param_req_sync[2];
        end
      end
    end
    wire pll_lock;
    reg [1:0] lock_sync = 2'b00;
    reg [20:0] phy_reset_count = 21'd0;
    wire phy_ready_50m = ddr_rst_n && lock_sync[1] && phy_reset_count[20];
    reg [1:0] eth_reset_sync = 2'b00;
    wire eth_rst_n_int = eth_reset_sync[1];

    // Bring the Ethernet clock domain out so the receive path can share this
    // PLL instead of instantiating a second one (the device has only 6).
    assign eth_clk125 = clk125;
    assign eth_rst_n  = eth_rst_n_int;

    eth_gigabit_pll u_eth_gigabit_pll (
        .refclk(sys_clk_50m), .clk125(clk125), .lock(pll_lock)
    );

    always @(posedge sys_clk_50m or negedge ddr_rst_n) begin
      if(!ddr_rst_n) begin lock_sync<=0;phy_reset_count<=0;end
      else begin
        lock_sync <= {lock_sync[0], pll_lock};
        if (!lock_sync[1])
            phy_reset_count <= 21'd0;
        else if (!phy_reset_count[20])
            phy_reset_count <= phy_reset_count + 1'b1;
      end
    end
    assign phy_reset_n = phy_ready_50m;

    always @(posedge clk125 or negedge phy_ready_50m) begin
        if (!phy_ready_50m)
            eth_reset_sync <= 2'b00;
        else
            eth_reset_sync <= {eth_reset_sync[0], 1'b1};
    end

    // Control states cross from the 100 MHz button layer into clk125. They are
    // quasi-static (they change only on a debounced press), so two flops are
    // enough and no handshake is needed.
    reg [2:0] ctrl_mode_s0, ctrl_mode_s1;
    reg [3:0] ctrl_flag_s0, ctrl_flag_s1;   // {detect, cal, ae, awb}
    reg [2:0] ctrl_man_s0, ctrl_man_s1;     // {dgain, again, exposure}
    always @(posedge clk125 or negedge eth_rst_n) begin
        if (!eth_rst_n) begin
            ctrl_mode_s0 <= 3'd0;
            ctrl_mode_s1 <= 3'd0;
            ctrl_flag_s0 <= 4'd0;
            ctrl_flag_s1 <= 4'd0;
            ctrl_man_s0  <= 3'd0;
            ctrl_man_s1  <= 3'd0;
        end else begin
            ctrl_mode_s0 <= ctrl_mode;
            ctrl_man_s0  <= {ctrl_manual_dgain, ctrl_manual_again,
                             ctrl_manual_exposure};
            ctrl_mode_s1 <= ctrl_mode_s0;
            ctrl_man_s1  <= ctrl_man_s0;
            ctrl_flag_s0 <= {ctrl_detect, ctrl_cal, ctrl_ae, ctrl_awb};
            ctrl_flag_s1 <= ctrl_flag_s0;
        end
    end

    function [24:0] frame_base_addr;
        input [1:0] slot;
        begin
            case (slot)
                2'd0: frame_base_addr = IMAGE_BASE_ADDR_0;
                2'd1: frame_base_addr = IMAGE_BASE_ADDR_1;
                2'd2: frame_base_addr = IMAGE_BASE_ADDR_2;
                default: frame_base_addr = IMAGE_BASE_ADDR_3;
            endcase
        end
    endfunction

    // Packetizer completion crosses back as a toggle. The active frame id and
    // frame_active level remain stable for the entire transfer.
    wire frame_done_toggle;
    reg [2:0] frame_done_sync;
    reg frame_done_seen;

    reg [15:0] source_frame_id;
    reg [2:0] warmup_frames;
    reg [7:0] frame_div_count;
    reg frame_active;
    reg [15:0] active_frame_id;
    reg [24:0] read_addr;
    reg [17:0] words_issued;
    reg [17:0] words_returned;
    // Wait for both established video clients and their responses to become
    // quiet before each short Ethernet burst.
    reg request_pending;
    reg burst_active;
    reg [6:0] burst_length;
    reg [6:0] burst_issued;
    reg [6:0] burst_returned;
    reg [5:0] cooldown_count;
    reg [15:0] pending_watchdog;
    reg [16:0] burst_watchdog;
    // Sticky diagnostic flag. It never terminates an in-progress frame.
    reg ethernet_fault;

    wire [127:0] fifo_dout;
    wire fifo_valid;
    wire fifo_full;
    wire fifo_empty;
    wire [8:0] fifo_wrusedw;
    wire [8:0] fifo_rdusedw;
    wire fifo_re;
    wire fifo_we = burst_active && ddr_user_rd_valid && !fifo_full;

    assign ddr_reserve = request_pending || burst_active;
    assign ddr_response_owner = burst_active;
    assign ddr_rd_en = burst_active &&
                       (burst_issued < burst_length) &&
                       ddr_user_ready;
    assign ddr_rd_addr = read_addr;

    always @(posedge ddr_clk or negedge ddr_rst_n) begin
        if (!ddr_rst_n) begin
            frame_done_sync <= 3'b000;
            frame_done_seen <= 1'b0;
            source_frame_id <= 16'd0;
            warmup_frames <= 3'd0;
            frame_div_count <= 8'd0;
            frame_active <= 1'b0;
            active_frame_id <= 16'd0;
            read_addr <= 25'd0;
            words_issued <= 18'd0;
            words_returned <= 18'd0;
            request_pending <= 1'b0;
            burst_active <= 1'b0;
            burst_length <= 7'd0;
            burst_issued <= 7'd0;
            burst_returned <= 7'd0;
            cooldown_count <= 6'd0;
            pending_watchdog <= 16'd0;
            burst_watchdog <= 17'd0;
            ethernet_fault <= 1'b0;
        end else begin
            frame_done_sync <= {frame_done_sync[1:0], frame_done_toggle};

            if (cooldown_count != 6'd0)
                cooldown_count <= cooldown_count - 1'b1;

            if (frame_done_sync[2] != frame_done_seen) begin
                frame_done_seen <= frame_done_sync[2];
                frame_active <= 1'b0;
            end

            // Wait until four camera frames have started before selecting a
            // ring-buffer slot; the first usable completed slot is then known
            // to contain a full frame. Thereafter select one source frame out
            // of FRAME_DIV. The raw default is explicitly 60/2 = 30 fps.
            if (ddr_frame_start) begin
                source_frame_id <= source_frame_id + 1'b1;
                if (warmup_frames != 3'd3) begin
                    warmup_frames <= warmup_frames + 1'b1;
                    // Prime the divider so the first safe completed frame is
                    // selected immediately after warm-up.
                    frame_div_count <= FRAME_DIV-1;
                end else if ((FRAME_DIV <= 1) ||
                             (frame_div_count == FRAME_DIV-1)) begin
                    frame_div_count <= 8'd0;
                    if (!frame_active) begin
                        frame_active <= 1'b1;
                        // Input names the preceding completed write slot, NOT
                        // the delayed HDMI ring pointer.
                        active_frame_id <= source_frame_id - 1'b1;
                        read_addr <= frame_base_addr(ddr_frame_slot);
                        words_issued <= 18'd0;
                        words_returned <= 18'd0;
                        request_pending <= 1'b0;
                        burst_active <= 1'b0;
                        cooldown_count <= 6'd0;
                        pending_watchdog <= 16'd0;
                        burst_watchdog <= 17'd0;
                    end
                end else begin
                    frame_div_count <= frame_div_count + 1'b1;
                end
            end

            if (!request_pending && !burst_active &&
                frame_active && (words_issued < FRAME_WORDS) &&
                (fifo_wrusedw <= 9'd320) &&
                (cooldown_count == 6'd0)) begin
                request_pending <= 1'b1;
                pending_watchdog <= 16'd0;
            end

            if (request_pending) begin
                // The top-level outstanding counter covers requests already
                // queued inside mc_to_user_interface and the DDR controller.
                // A low busy flag alone is not enough to transfer ownership.
                if (!ddr_video_wr_busy && !ddr_video_rd_busy &&
                    ddr_video_rd_idle) begin
                    request_pending <= 1'b0;
                    burst_active <= 1'b1;
                    burst_issued <= 7'd0;
                    burst_returned <= 7'd0;
                    pending_watchdog <= 16'd0;
                    burst_watchdog <= 17'd0;
                    if (words_issued <= FRAME_WORDS-BURST_WORDS)
                        burst_length <= BURST_WORDS;
                    else
                        burst_length <= FRAME_WORDS-words_issued;
                end else if (pending_watchdog >= PENDING_TIMEOUT_CYCLES) begin
                    // Before the first word, skipping the whole selected frame
                    // is safe. After transmission has started, release the
                    // reservation briefly and retry the same frame. Never
                    // convert a temporary arbitration delay into a half-frame.
                    request_pending <= 1'b0;
                    if (words_issued == 18'd0) begin
                        frame_active <= 1'b0;
                    end else begin
                        ethernet_fault <= 1'b1;
                        cooldown_count <= 6'd63;
                    end
                    pending_watchdog <= 16'd0;
                end else begin
                    pending_watchdog <= pending_watchdog + 1'b1;
                end
            end

            if (burst_active) begin
                if (ddr_rd_en || ddr_user_rd_valid) begin
                    burst_watchdog <= 17'd0;
                end else if (burst_watchdog >= BURST_TIMEOUT_CYCLES) begin
                    // Accepted reads own the next DDR responses. Relinquishing
                    // ownership here would mix Ethernet and HDMI data and tear
                    // the frame. Keep the burst active and resume on progress.
                    ethernet_fault <= 1'b1;
                    burst_watchdog <= 17'd0;
                end else begin
                    burst_watchdog <= burst_watchdog + 1'b1;
                end
            end

            if (ddr_rd_en) begin
                burst_issued <= burst_issued + 1'b1;
                words_issued <= words_issued + 1'b1;
                read_addr <= read_addr + 4'd8;
            end

            if (burst_active && ddr_user_rd_valid) begin
                burst_returned <= burst_returned + 1'b1;
                words_returned <= words_returned + 1'b1;
                if (burst_returned == burst_length-1'b1) begin
                    burst_active <= 1'b0;
                    cooldown_count <= 6'd32;
                    burst_watchdog <= 17'd0;
                end
            end
        end
    end

    w128_d512_fifo u_eth_video_fifo (
        .rst(~ddr_rst_n), .clkw(ddr_clk), .clkr(clk125),
        .we(fifo_we), .di(ddr_user_rd_data),
        .re(fifo_re), .dout(fifo_dout), .valid(fifo_valid),
        .full_flag(fifo_full), .empty_flag(fifo_empty),
        .afull(), .aempty(), .wrusedw(fifo_wrusedw),
        .rdusedw(fifo_rdusedw)
    );

    reg [1:0] frame_active_sync;
    reg [15:0] frame_id_meta;
    reg [15:0] frame_id_sync;
    always @(posedge clk125 or negedge eth_rst_n) begin
        if (!eth_rst_n) begin
            frame_active_sync <= 2'b00;
            frame_id_meta <= 16'd0;
            frame_id_sync <= 16'd0;
        end else begin
            frame_active_sync <= {frame_active_sync[0], frame_active};
            frame_id_meta <= active_frame_id;
            frame_id_sync <= frame_id_meta;
        end
    end

    wire [7:0] gmii_data;
    wire gmii_valid;
    reg tx_seen;
    reg read_seen;
    always @(posedge clk125 or negedge eth_rst_n) begin
        if(!eth_rst_n) tx_seen <= 1'b0;
        else if(gmii_valid) tx_seen <= 1'b1;
    end
    always @(posedge ddr_clk or negedge ddr_rst_n) begin
        if(!ddr_rst_n) read_seen <= 1'b0;
        else if(fifo_we) read_seen <= 1'b1;
    end
    // [3] fault, [2] PHY reset released/PLL locked (not negotiated link),
    // [1] received DDR data, [0] generated GMII bytes. Healthy activity = 7.
    assign debug_status = {ethernet_fault, eth_rst_n, read_seen, tx_seen};
    wire [127:0] packet_data;
    wire packet_valid,packet_empty,packet_re;
    wire [8:0] packet_count;
    generate if(COMPRESS_ENABLE) begin : compressed
        wire encode_we, encode_afull;
        wire [127:0] encode_data;
        rgb_block_encoder encoder(.clk(clk125),.rst_n(eth_rst_n),
            .in_data(fifo_dout),.in_valid(fifo_valid),.in_empty(fifo_empty),.in_re(fifo_re),
            .out_ready(!encode_afull),.out_we(encode_we),.out_data(encode_data));
        w128_d512_fifo encoded_fifo(.rst(!eth_rst_n),.clkw(clk125),.clkr(clk125),
            .we(encode_we),.di(encode_data),.afull(encode_afull),.full_flag(),.wrusedw(),
            .re(packet_re),.dout(packet_data),.valid(packet_valid),
            .empty_flag(packet_empty),.rdusedw(packet_count),.aempty());
    end else begin : raw
        assign fifo_re=packet_re;
        assign packet_data=fifo_dout;
        assign packet_valid=fifo_valid;
        assign packet_empty=fifo_empty;
        assign packet_count=fifo_rdusedw;
    end endgenerate
    udp_video_packetizer #(.JUMBO_ENABLE(0),.COMPRESS_ENABLE(COMPRESS_ENABLE)) u_packetizer (
        .clk(clk125), .rst_n(eth_rst_n),
        .fifo_data(packet_data), .fifo_valid(packet_valid),
        .fifo_empty(packet_empty), .fifo_count({1'b0, packet_count}),
        .frame_ready(frame_active_sync[1]), .frame_id(frame_id_sync),
        .ctrl_mode(ctrl_mode_s1),
        .ctrl_awb(ctrl_flag_s1[0]), .ctrl_ae(ctrl_flag_s1[1]),
        .ctrl_cal(ctrl_flag_s1[2]), .ctrl_detect(ctrl_flag_s1[3]),
        .ctrl_manual_exposure(ctrl_man_s1[0]),
        .ctrl_manual_again(ctrl_man_s1[1]),
        .ctrl_manual_dgain(ctrl_man_s1[2]),
        .applied_params(params_eth),
        .frame_done_toggle(frame_done_toggle),
        .fifo_re(packet_re), .gmii_data(gmii_data), .gmii_valid(gmii_valid)
    );

    rgmii_tx_out u_rgmii_tx_out (
        .clk125(clk125), .rst(~eth_rst_n),
        .gmii_data(gmii_data), .gmii_valid(gmii_valid),
        .rgmii_txc(rgmii_txc), .rgmii_txd(rgmii_txd),
        .rgmii_tx_ctl(rgmii_tx_ctl)
    );
endmodule
