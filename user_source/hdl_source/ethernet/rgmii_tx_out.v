// GMII byte stream to 1-Gbit/s RGMII. TXC delay is supplied by the pin
// constraint (OUTDEL=3), matching the vendor's board example.
module rgmii_tx_out (
    input  wire       clk125,
    input  wire       rst,
    input  wire [7:0] gmii_data,
    input  wire       gmii_valid,
    output wire       rgmii_txc,
    output wire [3:0] rgmii_txd,
    output wire       rgmii_tx_ctl
);
    PH1P_LOGIC_HD_ODDR u_txc (
        .q(rgmii_txc), .clk(clk125), .d0(1'b1), .d1(1'b0), .rst(rst)
    );

    PH1P_LOGIC_HD_ODDR u_ctl (
        .q(rgmii_tx_ctl), .clk(clk125),
        .d0(gmii_valid), .d1(gmii_valid), .rst(rst)
    );

    genvar i;
    generate
        for (i = 0; i < 4; i = i + 1) begin : g_txd
            PH1P_LOGIC_HD_ODDR u_txd (
                .q(rgmii_txd[i]), .clk(clk125),
                .d0(gmii_data[i]), .d1(gmii_data[i+4]), .rst(rst)
            );
        end
    endgenerate
endmodule
