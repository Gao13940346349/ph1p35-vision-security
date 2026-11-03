// 50 MHz -> 125 MHz clock used by the gigabit RGMII transmit path.
// The parameters are taken from the board vendor's YT8511 UDP example.
module eth_gigabit_pll (
    input  wire refclk,
    output wire clk125,
    output wire lock
);
    wire clk125_buf;

    PH1P_LOGIC_BUFG u_bufg_feedback (
        .i(clk125_buf),
        .o(clk125)
    );

    ph1p_phy_pll_wrapper_25a56e5ce2f9 #(
        .FBKCLK("CLKC0_EXT"), .FBKCLK_INT("CLKC0_EXT"),
        .FIN("50.000000000"), .REFCLK_DIV(2), .FBCLK_DIV(5),
        .CLKC0_ENABLE("ENABLE"), .CLKC0_DIV(7),
        .CLKC0_CPHASE(6), .CLKC0_FPHASE(0),
        .CLKC0_FPHASE_RSTSEL(0),
        .CLKC0_DUTY50("ENABLE"), .CLKC0_DUTY_INT(4),
        .PLL_USR_RST("ENABLE"), .PLL_FEED_TYPE("EXTERNAL"),
        .PLL_FASTLOOP("ENABLE"), .LPF_RES(0), .LPF_CAP(13), .ICP_CUR(29),
        .PHASE_PATH_SEL(0), .DYN_PHASE_PATH_SEL("DISABLE"),
        .DYN_FPHASE_EN("DISABLE"), .PI_OUT_SEL("NORMAL"),
        .PI_FRAC_EN("DISABLE"), .CLKC0_PI_SHIFT_EN("DISABLE"),
        .FEEDBK_MODE("NORMAL"), .DYN_CPHASE_EN("DISABLE")
    ) u_eth_pll (
        .refclk(refclk), .fbclk(clk125),
        .clk0_en(1'b1), .clk0_out(clk125_buf), .clkb0_out(),
        .clk1_en(1'b0), .clk1_out(), .clkb1_out(),
        .clk2_en(1'b0), .clk2_out(), .clkb2_out(),
        .clk3_en(1'b0), .clk3_out(), .clkb3_out(),
        .clk4_en(1'b0), .clk4_out(), .clkb4_out(),
        .clk5_en(1'b0), .clk5_out(), .clkb5_out(),
        .clk6_en(1'b0), .clk6_out(), .clkb6_out(),
        .lock(lock), .reset(1'b0), .clkc_rst(2'b00),
        .refclk_rst(1'b0), .wakeup(1'b1), .cps_step(1'b0),
        .pllpd(1'b0), .psclk(1'b0), .psclksel(3'b000),
        .psstep(1'b0), .psdown(1'b0), .psdone(),
        .drp_clk(1'b0), .drp_rstn(1'b1), .drp_sel(1'b0),
        .drp_rd(1'b0), .drp_wr(1'b0), .drp_addr(8'd0),
        .drp_wdata(8'd0), .drp_err(), .drp_rdy(), .drp_rdata(),
        .ssc_reset(1'b0)
    );
endmodule
