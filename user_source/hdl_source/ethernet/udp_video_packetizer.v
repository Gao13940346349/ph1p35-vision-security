// Transmit-only Ethernet/IPv4/UDP packetizer for a DDR-backed 1280x720 RGB888
// frame. Default v1 uses 1440-byte pixel payloads (1920 packets/frame).
// A frame is atomic after packet zero starts: frame_ready is deliberately not
// sampled again until all packets and the final FCS have been transmitted.
// Optional v2 uses jumbo Ethernet frames: one UDP datagram contains a
// 16-byte application header and 540 x 128-bit DDR words (2880 pixels / 8640
// bytes). Exactly 320 packets form one complete frame. This keeps RGB888
// 720p30 bit-exact while reducing the packet rate from 57,600 to 9,600 pkt/s.
//
// Application header bytes 4 and 5 pack the five control states plus the three
// manual-override flags into their high bits, with the low two bits of each
// byte staying version/pixel-format. This lets the host mirror the on-board
// buttons and mark which sliders it owns, without a second datagram:
//   byte4 = {again_man, exposure_man, detect, cal, ae, awb, version[1:0]}
//   byte5 = {dgain_man, telemetry[1:0], mode[2:0], pixel_format[1:0]}
// host/video_codec.py status_byte4/status_byte5 must match bit for bit.
module udp_video_packetizer #(
    parameter integer JUMBO_ENABLE = 0,
    parameter integer COMPRESS_ENABLE = 0,
    parameter [47:0] SRC_MAC = 48'h02_00_00_00_00_02,
    // Use a layer-2 broadcast so the bitstream is not tied to one PC/NIC.
    // DST_IP remains unicast: only the host configured as 192.168.137.1
    // accepts the UDP stream.
    parameter [47:0] DST_MAC = 48'hFF_FF_FF_FF_FF_FF,
    parameter [31:0] SRC_IP  = 32'hC0_A8_89_02,
    parameter [31:0] DST_IP  = 32'hC0_A8_89_01,
    parameter [15:0] SRC_PORT = 16'd16002,
    parameter [15:0] DST_PORT = 16'd16001
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire [127:0] fifo_data,
    input  wire         fifo_valid,
    input  wire         fifo_empty,
    input  wire [9:0]   fifo_count,
    input  wire         frame_ready,
    input  wire [15:0]  frame_id,
    // Control state echoed back in header bytes 4 and 5. It is already
    // synchronized into this clock domain by the caller and only changes on a
    // button press, so it is sampled combinationally at packet build time.
    input  wire [2:0]   ctrl_mode,
    input  wire         ctrl_awb,
    input  wire         ctrl_ae,
    input  wire         ctrl_cal,
    input  wire         ctrl_detect,
    // Which image-control parameters the host has taken over. Echoed so the
    // panel can mark its sliders. ACKed values use the two reserved bits below.
    input  wire         ctrl_manual_exposure,
    input  wire         ctrl_manual_again,
    input  wire         ctrl_manual_dgain,
    input  wire [63:0]  applied_params,
    output reg          frame_done_toggle,
    output reg          fifo_re,
    output reg  [7:0]   gmii_data,
    output reg          gmii_valid
);
    localparam integer WORDS_PER_PACKET = JUMBO_ENABLE ? 540 : 90;
    localparam [15:0] PACKETS_PER_FRAME = COMPRESS_ENABLE ? 960 : (JUMBO_ENABLE ? 320 : 1920);
    localparam [15:0] IP_TOTAL_LENGTH = WORDS_PER_PACKET * 16 + 44;
    localparam [15:0] UDP_LENGTH = WORDS_PER_PACKET * 16 + 24;
    // Constant-folded one's-complement sum, also valid when IPs change.
    localparam [31:0] IP_SUM = 32'h4500 + IP_TOTAL_LENGTH + 32'h4000 +
        32'h4011 + SRC_IP[31:16] + SRC_IP[15:0] + DST_IP[31:16] + DST_IP[15:0];
    localparam [31:0] IP_FOLD = (IP_SUM & 32'hFFFF) + (IP_SUM >> 16);
    localparam [15:0] IP_CHECKSUM = ~((IP_FOLD & 32'hFFFF) + (IP_FOLD >> 16));

    localparam [3:0] ST_WAIT     = 4'd0;
    localparam [3:0] ST_LOAD     = 4'd1;
    localparam [3:0] ST_PREAMBLE = 4'd2;
    localparam [3:0] ST_ETH      = 4'd3;
    localparam [3:0] ST_IP       = 4'd4;
    localparam [3:0] ST_UDP      = 4'd5;
    localparam [3:0] ST_APP      = 4'd6;
    localparam [3:0] ST_PIXELS   = 4'd7;
    localparam [3:0] ST_FCS      = 4'd8;
    localparam [3:0] ST_IFG      = 4'd9;

    reg [3:0] state;
    reg [10:0] byte_index;
    reg [9:0] word_index;
    reg [3:0] word_byte_index;
    reg [127:0] pixel_word;
    reg [15:0] packet_id;
    reg [15:0] packet_frame_id;
    reg [31:0] crc;
    reg [31:0] fcs;
    reg [3:0] ifg_count;
    reg [7:0] tx_byte;
    // 5600 clocks = 44.8 us between packet starts. At 1522 wire bytes this
    // bounds sustained wire use to 271.786 Mb/s, including preamble and IFG.
    reg [12:0] pace_count;
    reg frame_in_progress;
    // Use the two reserved header bits to transmit one atomic 64-bit snapshot
    // over each aligned group of 32 packets. Video layout remains unchanged.
    reg [63:0] param_snapshot;

    function [7:0] pixel_byte;
        input [127:0] data;
        input [3:0] index;
        begin
            case (index)
                4'd0:  pixel_byte = data[127:120];
                4'd1:  pixel_byte = data[119:112];
                4'd2:  pixel_byte = data[111:104];
                4'd3:  pixel_byte = data[103:96];
                4'd4:  pixel_byte = data[95:88];
                4'd5:  pixel_byte = data[87:80];
                4'd6:  pixel_byte = data[79:72];
                4'd7:  pixel_byte = data[71:64];
                4'd8:  pixel_byte = data[63:56];
                4'd9:  pixel_byte = data[55:48];
                4'd10: pixel_byte = data[47:40];
                4'd11: pixel_byte = data[39:32];
                4'd12: pixel_byte = data[31:24];
                4'd13: pixel_byte = data[23:16];
                4'd14: pixel_byte = data[15:8];
                default: pixel_byte = data[7:0];
            endcase
        end
    endfunction

    function [31:0] crc32_byte;
        input [31:0] crc_in;
        input [7:0] data;
        integer k;
        reg [31:0] c;
        begin
            c = crc_in;
            for (k = 0; k < 8; k = k + 1) begin
                if (c[0] ^ data[k])
                    c = (c >> 1) ^ 32'hEDB88320;
                else
                    c = c >> 1;
            end
            crc32_byte = c;
        end
    endfunction

    function [7:0] eth_header_byte;
        input [3:0] index;
        begin
            case (index)
                4'd0:  eth_header_byte = DST_MAC[47:40];
                4'd1:  eth_header_byte = DST_MAC[39:32];
                4'd2:  eth_header_byte = DST_MAC[31:24];
                4'd3:  eth_header_byte = DST_MAC[23:16];
                4'd4:  eth_header_byte = DST_MAC[15:8];
                4'd5:  eth_header_byte = DST_MAC[7:0];
                4'd6:  eth_header_byte = SRC_MAC[47:40];
                4'd7:  eth_header_byte = SRC_MAC[39:32];
                4'd8:  eth_header_byte = SRC_MAC[31:24];
                4'd9:  eth_header_byte = SRC_MAC[23:16];
                4'd10: eth_header_byte = SRC_MAC[15:8];
                4'd11: eth_header_byte = SRC_MAC[7:0];
                4'd12: eth_header_byte = 8'h08;
                default: eth_header_byte = 8'h00;
            endcase
        end
    endfunction

    function [7:0] ip_header_byte;
        input [4:0] index;
        begin
            case (index)
                5'd0: ip_header_byte = 8'h45;
                5'd1: ip_header_byte = 8'h00;
                5'd2: ip_header_byte = IP_TOTAL_LENGTH[15:8];
                5'd3: ip_header_byte = IP_TOTAL_LENGTH[7:0];
                5'd4,5'd5: ip_header_byte = 8'h00;
                5'd6: ip_header_byte = 8'h40;
                5'd7: ip_header_byte = 8'h00;
                5'd8: ip_header_byte = 8'h40;
                5'd9: ip_header_byte = 8'h11;
                5'd10: ip_header_byte = IP_CHECKSUM[15:8];
                5'd11: ip_header_byte = IP_CHECKSUM[7:0];
                5'd12: ip_header_byte = SRC_IP[31:24];
                5'd13: ip_header_byte = SRC_IP[23:16];
                5'd14: ip_header_byte = SRC_IP[15:8];
                5'd15: ip_header_byte = SRC_IP[7:0];
                5'd16: ip_header_byte = DST_IP[31:24];
                5'd17: ip_header_byte = DST_IP[23:16];
                5'd18: ip_header_byte = DST_IP[15:8];
                default: ip_header_byte = DST_IP[7:0];
            endcase
        end
    endfunction

    function [7:0] udp_header_byte;
        input [2:0] index;
        begin
            case (index)
                3'd0: udp_header_byte = SRC_PORT[15:8];
                3'd1: udp_header_byte = SRC_PORT[7:0];
                3'd2: udp_header_byte = DST_PORT[15:8];
                3'd3: udp_header_byte = DST_PORT[7:0];
                3'd4: udp_header_byte = UDP_LENGTH[15:8];
                3'd5: udp_header_byte = UDP_LENGTH[7:0];
                default: udp_header_byte = 8'h00;
            endcase
        end
    endfunction

    function [7:0] app_header_byte;
        input [3:0] index;
        begin
            case (index)
                4'd0: app_header_byte = 8'h46;
                4'd1: app_header_byte = 8'h50;
                4'd2: app_header_byte = 8'h47;
                4'd3: app_header_byte = 8'h56;
                4'd4: app_header_byte = {ctrl_manual_again, ctrl_manual_exposure,
                                         ctrl_detect, ctrl_cal, ctrl_ae, ctrl_awb,
                                         COMPRESS_ENABLE ? 2'd3 : (JUMBO_ENABLE ? 2'd2 : 2'd1)};
                4'd5: app_header_byte = {ctrl_manual_dgain, param_snapshot[63:62], ctrl_mode,
                                         COMPRESS_ENABLE ? 2'd2 : 2'd1};
                4'd6: app_header_byte = packet_frame_id[15:8];
                4'd7: app_header_byte = packet_frame_id[7:0];
                4'd8: app_header_byte = packet_id[15:8];
                4'd9: app_header_byte = packet_id[7:0];
                4'd10: app_header_byte = PACKETS_PER_FRAME[15:8];
                4'd11: app_header_byte = PACKETS_PER_FRAME[7:0];
                4'd12: app_header_byte = 8'h05;
                4'd13: app_header_byte = 8'h00;
                4'd14: app_header_byte = 8'h02;
                default: app_header_byte = 8'hD0;
            endcase
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_WAIT;
            fifo_re <= 1'b0;
            gmii_data <= 8'd0;
            gmii_valid <= 1'b0;
            frame_done_toggle <= 1'b0;
            byte_index <= 11'd0;
            word_index <= 10'd0;
            word_byte_index <= 4'd0;
            pixel_word <= 128'd0;
            packet_id <= 16'd0;
            packet_frame_id <= 16'd0;
            crc <= 32'hFFFF_FFFF;
            fcs <= 32'd0;
            ifg_count <= 4'd0;
            tx_byte <= 8'd0;
            pace_count <= 13'd0;
            frame_in_progress <= 1'b0;
            param_snapshot <= 64'd0;
        end else begin
            fifo_re <= 1'b0;
            gmii_valid <= 1'b0;
            gmii_data <= 8'd0;
            if(pace_count!=0) pace_count<=pace_count-1'b1;

            case (state)
                ST_WAIT: begin
                    if ((frame_in_progress || frame_ready) &&
                        !fifo_empty && pace_count==0 &&
                        (fifo_count >= WORDS_PER_PACKET)) begin
                        if (!frame_in_progress) begin
                            frame_in_progress <= 1'b1;
                            packet_frame_id <= frame_id;
                        end
                        fifo_re <= 1'b1;
                        if (packet_id[4:0] == 0)
                            param_snapshot <= applied_params;
                        if(COMPRESS_ENABLE) pace_count<=13'd5599;
                        state <= ST_LOAD;
                    end
                end

                ST_LOAD: begin
                    if (fifo_valid) begin
                        pixel_word <= fifo_data;
                        byte_index <= 11'd0;
                        state <= ST_PREAMBLE;
                    end
                end

                ST_PREAMBLE: begin
                    gmii_valid <= 1'b1;
                    gmii_data <= (byte_index == 11'd7) ? 8'hD5 : 8'h55;
                    if (byte_index == 11'd7) begin
                        byte_index <= 11'd0;
                        crc <= 32'hFFFF_FFFF;
                        state <= ST_ETH;
                    end else byte_index <= byte_index + 1'b1;
                end

                ST_ETH: begin
                    tx_byte = eth_header_byte(byte_index[3:0]);
                    gmii_valid <= 1'b1;
                    gmii_data <= tx_byte;
                    crc <= crc32_byte(crc, tx_byte);
                    if (byte_index == 11'd13) begin
                        byte_index <= 11'd0;
                        state <= ST_IP;
                    end else byte_index <= byte_index + 1'b1;
                end

                ST_IP: begin
                    tx_byte = ip_header_byte(byte_index[4:0]);
                    gmii_valid <= 1'b1;
                    gmii_data <= tx_byte;
                    crc <= crc32_byte(crc, tx_byte);
                    if (byte_index == 11'd19) begin
                        byte_index <= 11'd0;
                        state <= ST_UDP;
                    end else byte_index <= byte_index + 1'b1;
                end

                ST_UDP: begin
                    tx_byte = udp_header_byte(byte_index[2:0]);
                    gmii_valid <= 1'b1;
                    gmii_data <= tx_byte;
                    crc <= crc32_byte(crc, tx_byte);
                    if (byte_index == 11'd7) begin
                        byte_index <= 11'd0;
                        state <= ST_APP;
                    end else byte_index <= byte_index + 1'b1;
                end

                ST_APP: begin
                    tx_byte = app_header_byte(byte_index[3:0]);
                    gmii_valid <= 1'b1;
                    gmii_data <= tx_byte;
                    crc <= crc32_byte(crc, tx_byte);
                    if (byte_index == 11'd15) begin
                        word_index <= 10'd0;
                        word_byte_index <= 4'd0;
                        state <= ST_PIXELS;
                    end else byte_index <= byte_index + 1'b1;
                end

                ST_PIXELS: begin
                    tx_byte = pixel_byte(pixel_word, word_byte_index);
                    gmii_valid <= 1'b1;
                    gmii_data <= tx_byte;
                    crc <= crc32_byte(crc, tx_byte);

                    if ((word_byte_index == 4'd13) &&
                        (word_index != WORDS_PER_PACKET-1))
                        fifo_re <= 1'b1;

                    if (word_byte_index == 4'd15) begin
                        word_byte_index <= 4'd0;
                        if (word_index == WORDS_PER_PACKET-1) begin
                            fcs <= ~crc32_byte(crc, tx_byte);
                            byte_index <= 11'd0;
                            state <= ST_FCS;
                        end else begin
                            word_index <= word_index + 1'b1;
                            if (fifo_valid)
                                pixel_word <= fifo_data;
                        end
                    end else word_byte_index <= word_byte_index + 1'b1;
                end

                ST_FCS: begin
                    gmii_valid <= 1'b1;
                    case (byte_index[1:0])
                        2'd0: gmii_data <= fcs[7:0];
                        2'd1: gmii_data <= fcs[15:8];
                        2'd2: gmii_data <= fcs[23:16];
                        default: gmii_data <= fcs[31:24];
                    endcase
                    if (byte_index == 11'd3) begin
                        param_snapshot <= {param_snapshot[61:0], 2'b00};
                        if (packet_id == PACKETS_PER_FRAME-1) begin
                            packet_id <= 16'd0;
                            frame_in_progress <= 1'b0;
                            frame_done_toggle <= ~frame_done_toggle;
                        end else begin
                            packet_id <= packet_id + 1'b1;
                        end
                        ifg_count <= 4'd0;
                        state <= ST_IFG;
                    end else byte_index <= byte_index + 1'b1;
                end

                ST_IFG: begin
                    if (ifg_count == 4'd11)
                        state <= ST_WAIT;
                    else
                        ifg_count <= ifg_count + 1'b1;
                end

                default: state <= ST_WAIT;
            endcase
        end
    end
endmodule
