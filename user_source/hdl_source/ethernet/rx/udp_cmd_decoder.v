// Host command decoder: the only place a host -> FPGA command becomes state.
//
// The host broadcasts a 4-byte datagram from a pinned source port. This module
// accepts it only when all three of the following agree, so an unrelated UDP
// datagram on the same segment cannot move the image path:
//   - payload length is exactly 4
//   - first payload byte is the 0xA5 magic
//   - sender port is 16003
//
// Payload layout, matching host/video_codec.py encode_command():
//   byte0 = 0xA5 magic
//   byte1 = {1'b0, detect, cal, ae, awb, mode[2:0]}
//   byte2 = {5'b0, manual_en, param_sel[1:0]}
//   byte3 = parameter value
//
// byte1 is an absolute SET of the five on/off states, so repeating the
// datagram (the host sends three copies to survive a lost one) is idempotent.
// byte2/byte3 carry one image-control parameter at a time: param_sel picks
// which, manual_en says whether the host is taking it over or handing it back.
//
// uiudp_rx clears O_R_udp_len/O_R_udp_src_port on the same edge that drops
// O_R_udp_valid, so both are latched on the first payload byte while they are
// still valid.
//
// The commit is signalled as a TOGGLE, not a one-cycle pulse. This clock is
// 125 MHz and the consumer is 100 MHz, so an 8 ns strobe would be missed
// eventually. cmd_bus is registered in the same cycle the toggle flips, so the
// consumer's three-flop synchronizer has long since settled by the time it
// samples the bus after the edge.
module udp_cmd_decoder (
    input  wire        clk,
    input  wire        rst_n,
    // Received UDP payload from eth_control_rx (125 MHz domain)
    input  wire        r_udp_valid,
    input  wire [7:0]  r_udp_data,
    input  wire [15:0] r_udp_len,
    input  wire [15:0] r_udp_src_port,
    // Packed control bus and a toggle that flips once per accepted command
    output reg  [17:0] cmd_bus,
    output reg         cmd_toggle
);
    localparam [7:0]  CMD_MAGIC = 8'hA5;
    localparam [15:0] CMD_PORT  = 16'd16003;
    localparam [15:0] CMD_LEN   = 16'd4;

    reg [2:0]  byte_index;
    reg [7:0]  byte0;
    reg [7:0]  byte1;
    reg [7:0]  byte2;
    reg [7:0]  byte3;
    reg [15:0] latched_len;
    reg [15:0] latched_port;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            byte_index   <= 3'd0;
            byte0        <= 8'd0;
            byte1        <= 8'd0;
            byte2        <= 8'd0;
            byte3        <= 8'd0;
            latched_len  <= 16'd0;
            latched_port <= 16'd0;
            cmd_bus      <= 18'd0;
            cmd_toggle   <= 1'b0;
        end else begin
            if (r_udp_valid) begin
                if (byte_index == 3'd0) begin
                    byte0        <= r_udp_data;
                    latched_len  <= r_udp_len;
                    latched_port <= r_udp_src_port;
                end else if (byte_index == 3'd1) begin
                    byte1 <= r_udp_data;
                end else if (byte_index == 3'd2) begin
                    byte2 <= r_udp_data;
                end else begin
                    byte3 <= r_udp_data;   // byte 3 and any surplus
                end
                if (byte_index != 3'd7)
                    byte_index <= byte_index + 1'b1;
            end else if (byte_index != 3'd0) begin
                // Datagram finished. Commit only when every qualifier matched.
                byte_index <= 3'd0;
                if (byte0 == CMD_MAGIC &&
                    byte_index == 3'd4 && byte2[7:3] == 5'd0 && !byte1[7] &&
                    latched_len == CMD_LEN &&
                    latched_port == CMD_PORT) begin
                    // [2:0] mode, [6:3] flags (detect,cal,ae,awb),
                    // [8:7] param_sel, [9] manual_en, [17:10] param value
                    cmd_bus    <= {byte3, byte2[2],
                                   byte2[1:0], byte1[6:0]};
                    cmd_toggle <= ~cmd_toggle;
                end
            end
        end
    end
endmodule
