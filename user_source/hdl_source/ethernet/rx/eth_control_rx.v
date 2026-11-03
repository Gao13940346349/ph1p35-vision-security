// RGMII receive front end for the host control channel.
//
// The project's video path is transmit-only, so this is the only receive logic
// in the design. It is a receive-only port of the board vendor's (米联客)
// YT8511 reference stack, pruned to the minimum needed to hand out a UDP
// payload in the 125 MHz user clock domain:
//
//   eth_rgmii_rx  -> uimac_rx -> uiip_arp_rx -> uiip_rx -> uiudp_rx
//
// Deliberately not ported: the vendor transmit path, ARP, ICMP, MDIO and the
// MAC pause-frame parser. Two vendor filters were widened so a limited
// broadcast command can get through at all:
//   - uimac_rx: broadcast destination MAC now also accepts IPv4, not just ARP.
//   - uiip_rx : destination IP now also accepts 255.255.255.255.
//
// Every internal clock is the same 125 MHz user clock; the only real
// cross-domain boundary is the vendor FIFO inside uimac_rx, whose write side
// runs on the PHY-recovered clock.
//
// Reset is active-high internally, matching the vendor modules; the active-low
// system reset is inverted once, here.
module eth_control_rx (
    // RGMII pins
    input  wire [3:0] rgmii_rxd,
    input  wire       rgmii_rx_ctl,
    input  wire       rgmii_rxc,
    // 125 MHz user clock and active-low reset
    input  wire       clk125,
    input  wire       rst_n,
    // Received UDP payload (valid, data, payload length, sender port)
    output wire       r_udp_valid,
    output wire [7:0] r_udp_data,
    output wire [15:0] r_udp_len,
    output wire [15:0] r_udp_src_port
);
    localparam [47:0] MAC_LOCAL = 48'h02_00_00_00_00_02;
    localparam [31:0] IP_LOCAL  = 32'hC0_A8_89_02;   // 192.168.137.2

    // Vendor modules use an active-high, asynchronous reset.
    wire rx_reset = ~rst_n;

    wire        gmii_rx_clk;
    wire [7:0]  gmii_rxd;
    wire        gmii_rx_dv;
    wire        gmii_rx_er;

    wire        mac_rvalid;
    wire [7:0]  mac_rdata;
    wire [15:0] mac_rdata_type;
    wire        mac_rdata_error;

    wire        ip_rvalid;
    wire [7:0]  ip_rdata;
    wire        arp_rvalid;
    wire [7:0]  arp_rdata;

    wire        udp_ip_rvalid;
    wire [7:0]  udp_ip_rdata;

    eth_rgmii_rx u_eth_rgmii_rx (
        .rgmii_rxd    (rgmii_rxd),
        .rgmii_rx_ctl (rgmii_rx_ctl),
        .rgmii_rxc    (rgmii_rxc),
        .gmii_rx_clk  (gmii_rx_clk),
        .gmii_rxd     (gmii_rxd),
        .gmii_rx_dv   (gmii_rx_dv),
        .gmii_rx_er   (gmii_rx_er)
    );

    uimac_rx u_uimac_rx (
        .I_mac_local_addr  (MAC_LOCAL),
        .I_crc32_en        (1'b1),
        .I_reset           (rx_reset),
        .I_mac_rclk        (clk125),
        .O_mac_rvalid      (mac_rvalid),
        .O_mac_rdata       (mac_rdata),
        .O_mac_rdata_type  (mac_rdata_type),
        .O_mac_rdata_error (mac_rdata_error),
        .O_mac_pause_en    (),
        .O_mac_pause_time  (),
        .O_mac_pause_addr  (),
        .I_gmii_rclk       (gmii_rx_clk),
        .I_gmii_rvalid     (gmii_rx_dv),
        .I_gmii_rdata      (gmii_rxd)
    );

    uiip_arp_rx u_uiip_arp_rx (
        .I_ip_arp_reset   (rx_reset),
        .I_ip_arp_rclk    (clk125),
        .O_ip_rvalid      (ip_rvalid),
        .O_ip_rdata       (ip_rdata),
        .O_arp_rvalid     (arp_rvalid),
        .O_arp_rdata      (arp_rdata),
        .I_mac_rvalid     (mac_rvalid),
        .I_mac_rdata      (mac_rdata),
        .I_mac_rdata_type (mac_rdata_type)
    );

    uiip_rx u_uiip_rx (
        .I_ip_local_addr           (IP_LOCAL),
        .I_reset                   (rx_reset),
        .I_ip_clk                  (clk125),
        .I_ip_rvalid               (ip_rvalid),
        .I_ip_rdata                (ip_rdata),
        .O_icmp_req_ip_addr        (),
        .O_icmp_req_en             (),
        .O_icmp_req_id             (),
        .O_icmp_req_sq_num         (),
        .O_icmp_req_checksum       (),
        .O_icmp_ping_echo_data_valid(),
        .O_icmp_ping_echo_data     (),
        .O_icmp_ping_echo_data_len (),
        .O_udp_ip_rvalid           (udp_ip_rvalid),
        .O_udp_ip_rdata            (udp_ip_rdata),
        .O_checksum_rerror         ()
    );

    uiudp_rx u_uiudp_rx (
        .I_reset          (rx_reset),
        .I_R_udp_clk      (clk125),
        .O_R_udp_valid    (r_udp_valid),
        .O_R_udp_data     (r_udp_data),
        .O_R_udp_len      (r_udp_len),
        .O_R_udp_src_port (r_udp_src_port),
        .I_udp_ip_rvalid  (udp_ip_rvalid),
        .I_udp_ip_rdata   (udp_ip_rdata)
    );
endmodule
