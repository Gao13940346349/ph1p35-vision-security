// Switch synchronized/debounced in the ISP clock domain. Mode changes are
// committed only on AWB SOF, before video_proc receives the new frame's pixels.
module detect_control #(parameter integer DEBOUNCE_BITS=20)(
    input wire clk, rst_n, switch_n, sof,
    input wire [2:0] selected_mode,
    output reg enabled,
    output wire [2:0] mode
);
    reg sw0,sw1,stable;
    reg [DEBOUNCE_BITS-1:0] count;
    assign mode = enabled ? 3'd0 : selected_mode;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin sw0<=0; sw1<=0; stable<=0; count<=0; enabled<=0; end
        else begin
            sw0<=!switch_n; sw1<=sw0;
            if(sw1==stable) count<=0;
            else if(&count) begin stable<=sw1; count<=0; end
            else count<=count+1'b1;
            if(sof) enabled<=stable;
        end
    end
endmodule

// Bundled-data toggle mailbox: payload remains held until acknowledgement.
// A busy mailbox drops a new descriptor, never changes an in-flight payload.
module detect_mailbox #(parameter integer WIDTH=59, parameter integer LOCAL_RESETS=0)(
    input wire rst_n, src_clk, send,
    input wire [WIDTH-1:0] data_in,
    output wire ready,
    input wire dst_clk,
    output reg receive,
    output reg [WIDTH-1:0] data_out
);
    wire src_reset_n,dst_reset_n;
    generate if(LOCAL_RESETS) begin:g_local_resets
        local_reset_release u_src(.clk(src_clk),.async_reset_n(rst_n),.reset_n(src_reset_n));
        local_reset_release u_dst(.clk(dst_clk),.async_reset_n(rst_n),.reset_n(dst_reset_n));
    end else begin:g_shared_reset
        assign src_reset_n=rst_n;
        assign dst_reset_n=rst_n;
    end endgenerate
    reg [WIDTH-1:0] held;
    reg request,ack,ack0,ack1,req0,req1;
    assign ready=request==ack1;
    always @(posedge src_clk or negedge src_reset_n) begin
        if(!src_reset_n) begin request<=0; ack0<=0; ack1<=0; end
        else begin
            ack0<=ack; ack1<=ack0;
            if(send && ready) request<=!request;
        end
    end
    // Payload is inaccessible until a reset request/ack handshake completes.
    // No payload reset: avoid distributing reset to every wide descriptor bit.
    always @(posedge src_clk) begin
        if(src_reset_n && send && ready) held<=data_in;
    end
    always @(posedge dst_clk or negedge dst_reset_n) begin
        if(!dst_reset_n) begin req0<=0; req1<=0; ack<=0; receive<=0; data_out<=0; end
        else begin
            req0<=request; req1<=req0; receive<=0;
            if(req1!=ack) begin data_out<=held; receive<=1; ack<=req1; end
        end
    end
endmodule

// Match source-frame results to the ACTUAL DDR write/read slots. video_in
// writes slot 0 on its first SOF (its wp register increments on that edge).
// Descriptors arriving after a display read starts are deliberately not used
// until a later read of that same frame. Invalidation protects slot reuse.
module detect_frame_bridge #(parameter integer BOX_WIDTH=43)(
    input wire rst_n, camera_clk, result_strobe,
    input wire [BOX_WIDTH+15:0] result,
    input wire ddr_clk, write_start,
    input wire [1:0] write_slot,
    input wire read_start,
    input wire [1:0] read_slot,
    input wire hdmi_clk,
    output wire [BOX_WIDTH-1:0] display_box,
    output wire display_latency_valid,
    output wire display_latency_overflow,
    output wire [3:0] display_latency_frames,
    output wire read_new_frame, read_repeat_frame
);
    wire received;
    wire [BOX_WIDTH+15:0] incoming;
    detect_mailbox #(.WIDTH(BOX_WIDTH+16)) u_result(
        .rst_n(rst_n),.src_clk(camera_clk),.send(result_strobe),.data_in(result),.ready(),
        .dst_clk(ddr_clk),.receive(received),.data_out(incoming));
    reg [15:0] frame_id;
    reg [15:0] slot_id [0:3];
    // Do not reset/clear the payload array: that turns this wide cache into
    // thousands of resettable flops. Valid bits invalidate stale RAM contents.
    (* ram_style = "block" *) reg [BOX_WIDTH-1:0] slot_box [0:3];
    reg [BOX_WIDTH-1:0] slot_box_q;
    reg [3:0] box_valid;
    reg [3:0] assigned;
    wire [1:0] result_slot = incoming[BOX_WIDTH +:2]-2'd1;
    // The displayed frame's age is transported in the same mailbox as its
    // boxes, so the OSD can never combine metadata from different frames.
    // Age is one-based: the newest completed frame is shown as 1 frame.
    reg [5:0] outgoing_age;
    reg outgoing_box_valid;
    wire [BOX_WIDTH+5:0] outgoing = {outgoing_age,
        outgoing_box_valid ? slot_box_q : {BOX_WIDTH{1'b0}}};
    reg send_box;
    wire read_slot_valid = assigned[read_slot] &&
                           !(write_start && write_slot==read_slot);
    reg have_read;
    reg [15:0] last_read_id;
    assign read_new_frame = read_start && read_slot_valid &&
                           (!have_read || slot_id[read_slot]!=last_read_id);
    assign read_repeat_frame = read_start && read_slot_valid &&
                              have_read && slot_id[read_slot]==last_read_id;
    always @(posedge ddr_clk or negedge rst_n) begin
        if(!rst_n) begin have_read<=0; last_read_id<=0; end
        else if(read_start && read_slot_valid) begin
            have_read<=1; last_read_id<=slot_id[read_slot];
        end
    end
    wire [15:0] newest_frame_id = frame_id + (write_start ? 16'd1 : 16'd0);
    wire [15:0] read_frame_age = newest_frame_id - slot_id[read_slot] + 16'd1;
    wire accept_result = received && assigned[result_slot] &&
        slot_id[result_slot]==incoming[BOX_WIDTH +:16] &&
        !(write_start && write_slot==result_slot);
    always @(posedge ddr_clk) begin
        if(accept_result) slot_box[result_slot]<=incoming[BOX_WIDTH-1:0];
        // Read only on the event: the mailbox consumes this snapshot on the
        // next clock, even if read_slot changes immediately after read_start.
        if(read_start) slot_box_q<=slot_box[read_slot];
    end
    integer k;
    always @(posedge ddr_clk or negedge rst_n) begin
        if(!rst_n) begin
            frame_id<=0; assigned<=0; outgoing_age<=0; send_box<=0;
            box_valid<=0; outgoing_box_valid<=0;
            for(k=0;k<4;k=k+1) slot_id[k]<=0;
        end else begin
            send_box<=0;
            if(accept_result) box_valid[result_slot]<=1;
            if(write_start) begin
                frame_id<=frame_id+1'b1; slot_id[write_slot]<=frame_id+1'b1;
                assigned[write_slot]<=1; box_valid[write_slot]<=0;
            end
            if(read_start) begin
                outgoing_age<=read_slot_valid
                          ? {1'b1,(read_frame_age>16'd9),
                             (read_frame_age>16'd9 ? 4'd9 : read_frame_age[3:0])}
                          : 6'd0;
                outgoing_box_valid<=read_slot_valid && box_valid[read_slot];
                send_box<=1;
            end
        end
    end
    // read_start comes from video_out's VSYNC edge in vertical blanking.
    // This mailbox updates well before active video and then holds for a frame.
    wire [BOX_WIDTH+5:0] displayed;
    assign display_box = displayed[BOX_WIDTH-1:0];
    assign display_latency_frames = displayed[BOX_WIDTH +: 4];
    assign display_latency_overflow = displayed[BOX_WIDTH+4];
    assign display_latency_valid = displayed[BOX_WIDTH+5];
    detect_mailbox #(.WIDTH(BOX_WIDTH+6)) u_display(
        .rst_n(rst_n),.src_clk(ddr_clk),.send(send_box),.data_in(outgoing),.ready(),
        .dst_clk(hdmi_clk),.receive(),.data_out(displayed));
endmodule

// Event rates over an exact sys-clock second. Independent event toggles cross
// domains; events must be separated by at least 3 sys clocks (video frames are).
// CAM means admitted DDR write SOFs, not a claim of complete sensor frames.
module video_frame_statistics #(parameter integer SYS_HZ=50000000)(
    input wire rst_n, sys_clk, ddr_clk, camera_clk, hdmi_clk,
    input wire write_start, read_new_frame, read_repeat_frame,
    input wire detect_result, hdmi_frame,
    output reg [60:0] display_stats
);
    reg [2:0] ddr_toggle;
    reg camera_toggle, hdmi_toggle;
    always @(posedge ddr_clk or negedge rst_n) begin
        if(!rst_n) ddr_toggle<=0;
        else ddr_toggle<=ddr_toggle ^ {read_repeat_frame,read_new_frame,write_start};
    end
    always @(posedge camera_clk or negedge rst_n) begin
        if(!rst_n) camera_toggle<=0;
        else if(detect_result) camera_toggle<=!camera_toggle;
    end
    always @(posedge hdmi_clk or negedge rst_n) begin
        if(!rst_n) hdmi_toggle<=0;
        else if(hdmi_frame) hdmi_toggle<=!hdmi_toggle;
    end
    (* async_reg="true" *) reg [4:0] sync0,sync1;
    reg [4:0] seen;
    wire [4:0] events=sync1 ^ seen;
    localparam integer SECOND_BITS=(SYS_HZ<2)?1:$clog2(SYS_HZ);
    reg [SECOND_BITS-1:0] second_count;
    reg [59:0] counts;
    wire [59:0] counts_next;
    wire send_snapshot=second_count==SYS_HZ-1;
    wire [60:0] received_stats;
    // Three decimal digits, saturated at 999; no divider or combinational
    // binary-to-decimal network is placed on the pixel path.
    function [11:0] inc_bcd;
        input [11:0] value;
        begin
            inc_bcd=value;
            if(value!=12'h999) begin
                if(value[3:0]!=9) inc_bcd[3:0]=value[3:0]+1'b1;
                else begin
                    inc_bcd[3:0]=0;
                    if(value[7:4]!=9) inc_bcd[7:4]=value[7:4]+1'b1;
                    else begin inc_bcd[7:4]=0; inc_bcd[11:8]=value[11:8]+1'b1; end
                end
            end
        end
    endfunction
    genvar g;
    generate for(g=0;g<5;g=g+1) begin:g_count_next
        assign counts_next[g*12+:12]=events[g] ? inc_bcd(counts[g*12+:12]) : counts[g*12+:12];
    end endgenerate
    always @(posedge sys_clk or negedge rst_n) begin
        if(!rst_n) begin
            sync0<=0;sync1<=0;seen<=0;second_count<=0;
            counts<=0;
        end else begin
            // Packed fields low to high: CAM, NEW, REP, HDMI, DET.
            sync0<={camera_toggle,hdmi_toggle,ddr_toggle};
            sync1<=sync0; seen<=sync1;
            if(send_snapshot) begin
                second_count<=0; counts<=0;
            end else begin
                second_count<=second_count+1'b1; counts<=counts_next;
            end
        end
    end
    // Capture the closing window directly in the mailbox on the same edge
    // that clears the counters, saving a redundant 60-bit snapshot register.
    detect_mailbox #(.WIDTH(61)) u_stats_mailbox(
        .rst_n(rst_n),.src_clk(sys_clk),.send(send_snapshot),
        .data_in({1'b1,counts_next}),.ready(),.dst_clk(hdmi_clk),
        .receive(),.data_out(received_stats));
    // Freeze all numbers together for the entire displayed frame.
    always @(posedge hdmi_clk or negedge rst_n) begin
        if(!rst_n) display_stats<=0;
        else if(hdmi_frame) display_stats<=received_stats;
    end
endmodule
