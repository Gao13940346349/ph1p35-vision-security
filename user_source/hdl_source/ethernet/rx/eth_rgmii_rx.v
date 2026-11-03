// RGMII receive -> GMII receive converter.
//
// Adapted from the board vendor's rgmii_interface.v (YT8511/米联客 reference
// design), keeping only the receive half. The vendor module also contained the
// transmit half; the project already has its own transmit path (rgmii_tx_out.v
// plus udp_video_packetizer.v), so that half is deliberately not duplicated
// here and the vendor ODDR instances are not ported.
//
// Pin-side primitives are Anlogic PH1P LogicHD; the recovered RXC is buffered
// for the DDR capture and brought out as a PHY-local clock so uimac_rx's
// cross-domain FIFO can use it as its write clock.
module eth_rgmii_rx #(
    // Dedicated HD-bank delay taps, not LUT/inverter delay chains. Initial
    // setting must be checked at every STA corner after place/route.
    parameter [4:0] RX_INPUT_DELAY_TAPS = 5'd20
) (
    // RGMII pins
    input  wire [3:0] rgmii_rxd,
    input  wire       rgmii_rx_ctl,
    input  wire       rgmii_rxc,
    // Recovered receive clock (PHY domain, 125 MHz at gigabit)
    output wire       gmii_rx_clk,
    // GMII receive bus, registered on gmii_rx_clk
    output reg  [7:0] gmii_rxd,
    output reg        gmii_rx_dv,
    output reg        gmii_rx_er
);
    wire       rgmii_rxc_int;
    wire [3:0] rgmii_rxd_delay;
    wire rgmii_rx_ctl_delay;
    wire [7:0] gmii_rxd_reg;
    wire       gmii_rx_dv_reg;
    wire       rgmii_rx_ctl_reg;
    wire       gmii_rx_er_reg;

    // Clock the DDR capture from a global buffer.
    PH1P_LOGIC_BUFG u_rxc_bufg (
        .o (rgmii_rxc_int),
        .i (rgmii_rxc)
    );

    // PHY-local clock for the downstream cross-domain FIFO write side.
    PH1P_PHY_LCLK u_gmii_rx_clk (
        .ce        (1'b1        ),
        .clkin     (rgmii_rxc   ),
        .rst       (1'b0        ),
        .clkout    (gmii_rx_clk ),
        .clkdivout (            )
    );

    genvar j;
    generate for (j = 0; j < 4; j = j + 1) begin : rxdata_bus
        PH1P_LOGIC_DYNAMIC_HD_IDELAY #(.CAS_MODE("NONE")) u_rx_delay (
            .i(rgmii_rxd[j]), .o(rgmii_rxd_delay[j]),
            .dly(RX_INPUT_DELAY_TAPS),
            .cas_i(1'b0), .cas_rtn_i(1'b0), .cas_o(), .cas_rtn_o()
        );
    end
    endgenerate

    PH1P_LOGIC_DYNAMIC_HD_IDELAY #(.CAS_MODE("NONE")) u_ctl_delay (
        .i(rgmii_rx_ctl), .o(rgmii_rx_ctl_delay),
        .dly(RX_INPUT_DELAY_TAPS),
        .cas_i(1'b0), .cas_rtn_i(1'b0), .cas_o(), .cas_rtn_o()
    );

    // 1 Gbps: 4 bits per RGMII edge, so q0 is the rising-edge nibble and q1
    // the falling-edge nibble of one GMII byte.
    genvar k;
    generate for (k = 0; k < 4; k = k + 1) begin : rxdata_in_bus
        PH1P_LOGIC_HD_IDDR rgmii_rx_data_in (
            .q0  (gmii_rxd_reg[k]),
            .q1  (gmii_rxd_reg[k+4]),
            .clk (rgmii_rxc_int),
            .d   (rgmii_rxd_delay[k]),
            .rst (1'b0)
        );
    end
    endgenerate

    PH1P_LOGIC_HD_IDDR rgmii_rx_ctl_in (
        .q0  (gmii_rx_dv_reg),
        .q1  (rgmii_rx_ctl_reg),
        .clk (rgmii_rxc_int),
        .d   (rgmii_rx_ctl_delay),
        .rst (1'b0)
    );

    assign gmii_rx_er_reg = gmii_rx_dv_reg ^ rgmii_rx_ctl_reg;

    always @(posedge gmii_rx_clk) begin
        gmii_rxd   <= gmii_rxd_reg;
        gmii_rx_dv <= gmii_rx_dv_reg;
        gmii_rx_er <= gmii_rx_er_reg;
    end
endmodule
