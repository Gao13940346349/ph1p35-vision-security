// Side-channel detector. No ready/backpressure and no changes to the RGB stream.
// 4-connected same-colour regions on a 1/4-resolution two-bit colour mask.
// Nine object slots: independent components, including repeated colour/shape.
// Inputs: four RGB888 pixels per valid beat, P0 at [95:72], early SOF allowed.
module color_shape_detector #(
    parameter integer IMG_WIDTH = 1280,
    parameter integer IMG_HEIGHT = 720,
    parameter integer MIN_AREA = 120,
    parameter integer MIN_SIZE = 16,
    parameter integer SPATIAL_FILTER = 1
)(
    input wire clk, rst_n, enable, sof, valid,
    input wire [95:0] rgb,
    output reg result_strobe,
    output reg [15:0] result_id,
    output wire [15:0] capture_frame_id,
    // Slot k at [k*47 +:47]: colour2, shape2, valid, xmin11, xmax11, ymin10, ymax10.
    // Colour: 1 red, 2 green, 3 blue. Shape: 0 triangle, 1 square, 2 circle.
    output reg [422:0] boxes,
    output wire busy,
    // Capacity warning; boxes contain only complete, qualified components.
    // This is not a frame-invalid flag for downstream tracking.
    output reg overflow
);
    localparam W = IMG_WIDTH/4, H = IMG_HEIGHT/4, N = W*H;
    localparam IDLE=0, INIT=1, FETCH=2, MASK=3, LEFT=4, UP=5,
        SELECT=6, READ=7, WRITE=8, ADVANCE=9, MERGE=10,
        ROOT=11, CHILD_READ=12, CHILD_SAVE=13, ROOT_READ=14,
        MERGE_WRITE=15, MERGE_NEXT=16, EVAL=17, EVAL_READ=18,
        EVAL_PRODUCT=19, EVAL_TEST=20, EVAL_NEXT=21, DONE=22,
        EVAL_AXIS=23, EVAL_DIAG=24, PICK_SCAN=25, PICK_WRITE=26,
        LEFT_REQ=27, UP_REQ=28, ROOT_REQ=29, EVAL_DECIDE=30, EVAL_MUL_WAIT=31;
    reg [4:0] state;
    assign busy = state != IDLE;

    // Camera-tolerant thresholds: dominant >=32, margin >=12, ratio >=9/8.
    // Reject near-neutral pixels and ties instead of favouring a colour.
    function [1:0] pixel_colour;
        input [23:0] p;
        reg [10:0] r,g,b;
        begin
            r={3'd0,p[23:16]}; g={3'd0,p[15:8]}; b={3'd0,p[7:0]};
            if(r>=32 && r>=g+12 && r>=b+12 && (r<<3)>=(g<<3)+g && (r<<3)>=(b<<3)+b)
                pixel_colour=1;
            else if(g>=32 && g>=r+12 && g>=b+12 && (g<<3)>=(r<<3)+r && (g<<3)>=(b<<3)+b)
                pixel_colour=2;
            else if(b>=32 && b>=r+12 && b>=g+12 && (b<<3)>=(r<<3)+r && (b<<3)>=(g<<3)+g)
                pixel_colour=3;
            else pixel_colour=0;
        end
    endfunction
    reg [7:0] colour_bits;
    wire [2:0] red_votes = {2'd0,colour_bits[7:6]==1}+{2'd0,colour_bits[5:4]==1}+
                          {2'd0,colour_bits[3:2]==1}+{2'd0,colour_bits[1:0]==1};
    wire [2:0] green_votes = {2'd0,colour_bits[7:6]==2}+{2'd0,colour_bits[5:4]==2}+
                            {2'd0,colour_bits[3:2]==2}+{2'd0,colour_bits[1:0]==2};
    wire [2:0] blue_votes = {2'd0,colour_bits[7:6]==3}+{2'd0,colour_bits[5:4]==3}+
                           {2'd0,colour_bits[3:2]==3}+{2'd0,colour_bits[1:0]==3};
    // A tied 2+2 group is ambiguous: discard it rather than favour red.
    wire [1:0] voted_colour = red_votes>=2 && red_votes>green_votes && red_votes>blue_votes ? 2'd1 :
        green_votes>=2 && green_votes>red_votes && green_votes>blue_votes ? 2'd2 :
        blue_votes>=2 && blue_votes>red_votes && blue_votes>green_votes ? 2'd3 : 2'd0;
    reg cap_bank, proc_bank, capturing;
    reg [8:0] cx;
    reg [9:0] cy;
    reg [15:0] cap_addr, frame_id, work_id;
    assign capture_frame_id = frame_id;
    reg sof_d;
    wire start_frame = sof && !sof_d;
    wire end_frame = capturing && valid && cx==W-1 && cy==IMG_HEIGHT-1;
    wire capture_write = capturing && valid && cy[1:0]==2 && !start_frame;
    reg capture_we_d, cap_bank_d;
    reg [15:0] cap_addr_d;
    reg [8:0] cap_x_d;
    reg cap_first_row_d;
    // Separate RGB classification from horizontal voting/RAM write timing.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin colour_bits<=0; capture_we_d<=0; cap_bank_d<=0; cap_addr_d<=0; cap_x_d<=0;cap_first_row_d<=0;end
        else begin
            colour_bits<={pixel_colour(rgb[95:72]),pixel_colour(rgb[71:48]),pixel_colour(rgb[47:24]),pixel_colour(rgb[23:0])};
            capture_we_d<=capture_write; cap_bank_d<=cap_bank; cap_addr_d<=cap_addr;
            cap_x_d<=cx;cap_first_row_d<=cy<4;
        end
    end
    // 2x2 same-colour majority BEFORE label allocation, not after overflow.
    // Removes isolated points/thin toolbar strokes; retains the large interior
    // of all shapes. One 640-bit raw-mask line, no full-frame RGB buffer.
    // RAM is read-before-write. First row/column use explicit zero padding,
    // so disabled/truncated frames cannot leak stale line RAM into a new frame.
    wire [1:0] upper_q;
    reg [1:0] raw_q,left_raw,left_upper;
    reg filtered_we,filtered_bank,first_row_q,first_col_q;
    reg [15:0] filtered_addr;
    detector_eram #(.DATA_WIDTH(2),.ADDR_WIDTH(9),.DEPTH(W),.SINGLE_PORT(1)) u_previous_row(
        .clk(clk),.ce(capture_we_d),.we(capture_we_d),.waddr(cap_x_d),
        .wdata(voted_colour),.raddr(cap_x_d),.q(upper_q));
    wire [1:0] up_c=first_row_q ? 2'd0 : upper_q;
    wire [1:0] left_c=first_col_q ? 2'd0 : left_raw;
    wire [1:0] ul_c=(first_row_q || first_col_q) ? 2'd0 : left_upper;
    function [1:0] majority4;
        input [1:0] a,b,c,d;
        begin
            if(a!=0 && ((a==b && a==c)||(a==b && a==d)||(a==c && a==d))) majority4=a;
            else if(b!=0 && b==c && b==d) majority4=b;
            else majority4=0;
        end
    endfunction
    wire [1:0] filtered_colour=SPATIAL_FILTER ? majority4(raw_q,up_c,left_c,ul_c) : raw_q;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            filtered_we<=0;filtered_bank<=0;filtered_addr<=0;raw_q<=0;
            first_row_q<=0;first_col_q<=0;left_raw<=0;left_upper<=0;
        end else begin
            filtered_we<=capture_we_d;filtered_bank<=cap_bank_d;filtered_addr<=cap_addr_d;
            raw_q<=voted_colour;first_row_q<=cap_first_row_d;first_col_q<=cap_x_d==0;
            if(filtered_we) begin left_raw<=raw_q;left_upper<=up_c;end
        end
    end
    reg [15:0] scan_addr;
    wire [1:0] mask_q0, mask_q1;
    wire [1:0] sample_colour = proc_bank ? mask_q1 : mask_q0;
    reg [1:0] current_colour, left_colour;
    // Synchronous read, no RAM reset/clear: every captured mask cell is overwritten.
    detector_eram #(.DATA_WIDTH(2),.ADDR_WIDTH(16),.DEPTH(N)) u_mask0(
        .clk(clk),.ce(1'b1),.we(filtered_we && !filtered_bank),.waddr(filtered_addr),
        .wdata(filtered_colour),.raddr(scan_addr),.q(mask_q0));
    detector_eram #(.DATA_WIDTH(2),.ADDR_WIDTH(16),.DEPTH(N)) u_mask1(
        .clk(clk),.ce(1'b1),.we(filtered_we && filtered_bank),.waddr(filtered_addr),
        .wdata(filtered_colour),.raddr(scan_addr),.q(mask_q1));
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            cap_bank<=0; capturing<=0; cx<=0; cy<=0; cap_addr<=0;
            frame_id<=0; sof_d<=0;
        end else begin
            sof_d <= sof;
            if(start_frame) begin
                frame_id<=frame_id+1'b1;
                cap_bank<=busy ? !proc_bank : !cap_bank;
                capturing<=enable; cx<=valid ? 1 : 0; cy<=0; cap_addr<=0;
            end else if(capturing && valid) begin
                if(cy[1:0]==2) cap_addr<=cap_addr+1'b1;
                if(cx==W-1) begin cx<=0; cy<=cy+1'b1; end
                else cx<=cx+1'b1;
                if(end_frame) capturing<=0;
            end
        end
    end

    // Bounded union-find label table. Label 0 means background. The former
    // 127-label limit was too small for static textured scenes; 1023 labels
    // preserve exact target boundaries while tolerating many colour fragments.
    // One synchronous ERAM port, shared by all union-find lookups.
    // Multiple asynchronous array reads expanded into LUT muxes and registers.
    reg [9:0] left_label, left_root, up_root, root;
    reg [10:0] next_label, idx;
    wire [9:0] parent_q;
    wire parent_union = state==SELECT && left_root!=0 && up_root!=0 && left_root!=up_root;
    wire parent_we = state==INIT || parent_union;
    wire [9:0] parent_addr = state==INIT || state==EVAL || state==EVAL_READ ? idx[9:0] :
        parent_union ? (left_root<up_root ? up_root : left_root) :
        state==LEFT_REQ || state==LEFT ? left_root :
        state==UP_REQ || state==UP ? up_root : root;
    wire [9:0] parent_data = state==INIT ? idx[9:0] :
        (left_root<up_root ? left_root : up_root);
    PH1P_LOGIC_ERAM #(
        .DATA_WIDTH_A(10),.DATA_WIDTH_B(10),.ADDR_WIDTH_A(10),.ADDR_WIDTH_B(10),
        .DATA_DEPTH_A(1024),.DATA_DEPTH_B(1024),.MODE("SP"),
        .REGMODE_A("NOREG"),.REGMODE_B("NOREG"),.IMPLEMENT("20K"),
        .RESETMODE_A("ASYNC"),.OREGSET_A("RESET"),.ASYNC_RESET_RELEASE_A("ASYNC")
    ) u_parent_ram(
        .clka(clk),.cea(1'b1),.ocea(1'b1),.rsta(!rst_n),
        .wea(parent_we),.bea(1'b0),.dia(parent_data),.addra(parent_addr),.doa(parent_q),
        .clkb(clk),.ceb(1'b0),.oceb(1'b0),.rstb(1'b0),
        .web(1'b0),.beb(1'b0),.dib(10'd0),.addrb(10'd0),.dob(),
        .ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0),.ecc_sbiterr(),.ecc_dbiterr()
    );
    wire [11:0] line_q;
    reg [8:0] px;
    reg [7:0] py;
    reg [7:0] first_dropped_y;
    // An empty mask row separates all 4-connected components. Finalize that
    // band and recycle its labels; retain the frame's top-nine candidates.
    reg row_has_colour, resume_scan;
    // Packed: colour2, umin9, umax9, vmin9, vmax9, area16, xmin9, xmax9, ymin8, ymax8.
    // Diagonal coordinates u=x+y and v=x+(H-1)-y avoid signed arithmetic.
    wire [87:0] stats_q;
    reg [87:0] child;
    wire [15:0] a = stats_q[49:34];
    localparam [8:0] V_BIAS=H-1;
    wire [8:0] pu=px+{1'b0,py}, pv=px+V_BIAS-{1'b0,py};
    wire [8:0] ul=stats_q[85:77], ur=stats_q[76:68];
    wire [8:0] vl=stats_q[67:59], vr=stats_q[58:50];
    wire [8:0] merged_ul=a==0 || child[85:77]<ul ? child[85:77] : ul;
    wire [8:0] merged_ur=a==0 || child[76:68]>ur ? child[76:68] : ur;
    wire [8:0] merged_vl=a==0 || child[67:59]<vl ? child[67:59] : vl;
    wire [8:0] merged_vr=a==0 || child[58:50]>vr ? child[58:50] : vr;
    wire [9:0] stats_addr = (state==INIT || state==CHILD_READ ||
        state==CHILD_SAVE || state==EVAL || state==EVAL_READ) ? idx[9:0] : root;
    wire [8:0] xl = stats_q[33:25], xr = stats_q[24:16];
    wire [7:0] yt = stats_q[15:8], yb = stats_q[7:0];
    wire [15:0] merged_a = a + child[49:34];
    wire [8:0] merged_xl = a==0 || child[33:25]<xl ? child[33:25] : xl;
    wire [8:0] merged_xr = a==0 || child[24:16]>xr ? child[24:16] : xr;
    wire [7:0] merged_yt = a==0 || child[15:8]<yt ? child[15:8] : yt;
    wire [7:0] merged_yb = a==0 || child[7:0]>yb ? child[7:0] : yb;
    wire stats_we = state==INIT || state==WRITE || (state==MERGE_WRITE && child[49:34]!=0);
    wire [9:0] stats_waddr = state==INIT ? idx[9:0] : root;
    wire [87:0] stats_wdata = state==INIT ? 88'd0 : state==WRITE ?
        {current_colour, (a==0 || pu<ul ? pu : ul), (a==0 || pu>ur ? pu : ur),
         (a==0 || pv<vl ? pv : vl), (a==0 || pv>vr ? pv : vr), a+16'd1, (a==0 || px<xl ? px : xl), (a==0 || px>xr ? px : xr),
         (a==0 || py<yt ? py : yt), (a==0 || py>yb ? py : yb)} :
        (a==0 ? {child[87:86],merged_ul,merged_ur,merged_vl,merged_vr,merged_a,merged_xl,merged_xr,merged_yt,merged_yb} :
        {stats_q[87:86],merged_ul,merged_ur,merged_vl,merged_vr,merged_a,merged_xl,merged_xr,merged_yt,merged_yb});
    detector_eram #(.DATA_WIDTH(12),.ADDR_WIDTH(9),.DEPTH(W),.SINGLE_PORT(1)) u_line_label(
        .clk(clk),.ce(1'b1),.we(state==ADVANCE),.waddr(px),
        .wdata({current_colour,root}),.raddr(px),.q(line_q));
    // Writes only occur in INIT/WRITE/MERGE_WRITE: the read and write
    // addresses coincide in all three states. One read-before-write port
    // preserves the original one-clock access without a dual-address mux.
    detector_eram #(.DATA_WIDTH(88),.ADDR_WIDTH(10),.DEPTH(1024),.SINGLE_PORT(1)) u_stats(
        .clk(clk),.ce(1'b1),.we(stats_we),.waddr(stats_waddr),
        .wdata(stats_wdata),.raddr(stats_addr),.q(stats_q));
    reg [15:0] best_area [0:8];
    reg [46:0] best_box [0:8];
    reg [3:0] pick_index, pick_slot;
    reg [15:0] pick_area;
    // Keep the tiny top-nine table as nine fixed register lanes. Variable
    // array reads/writes can expand into wide index/shift networks in TD's
    // macro optimizer; explicit lane decode bounds that intermediate logic.
    reg [15:0] scanned_area;
    always @(*) begin
        case(pick_index)
            4'd0: scanned_area=best_area[0];
            4'd1: scanned_area=best_area[1];
            4'd2: scanned_area=best_area[2];
            4'd3: scanned_area=best_area[3];
            4'd4: scanned_area=best_area[4];
            4'd5: scanned_area=best_area[5];
            4'd6: scanned_area=best_area[6];
            4'd7: scanned_area=best_area[7];
            4'd8: scanned_area=best_area[8];
            default: scanned_area=16'd0;
        endcase
    end
    reg [19:0] rect_area,diag_area;
    reg [87:0] candidate;
    wire [9:0] bw = {1'b0,xr}-{1'b0,xl}+10'd1;
    wire [9:0] bh = {2'b00,yb}-{2'b00,yt}+10'd1;
    wire [9:0] du = {1'b0,candidate[76:68]}-{1'b0,candidate[85:77]}+10'd1;
    wire [9:0] dv = {1'b0,candidate[58:50]}-{1'b0,candidate[67:59]}+10'd1;
    wire [9:0] mul_x = state==EVAL_PRODUCT ? bw : du;
    wire [9:0] mul_y = state==EVAL_PRODUCT ? bh : dv;
    reg [9:0] mul_x_q,mul_y_q;
    reg [19:0] product;
    always @(posedge clk) begin mul_x_q<=mul_x;mul_y_q<=mul_y;end
    always @(posedge clk) product<=mul_x_q*mul_y_q; // one shared, registered multiplier
    wire [21:0] score={1'b0,rect_area,1'b0}+{2'b00,diag_area};
    wire [25:0] score16={score,4'd0};
    wire [25:0] area_ext={10'd0,candidate[49:34]};
    wire [26:0] area128={4'd0,candidate[49:34],7'd0};
    wire [26:0] rect_ext={7'd0,rect_area};
    // Exact constant products shared between the three classifiers. Keep
    // the thresholds/widths unchanged, but avoid long repeated adder chains.
    wire [25:0] area3=(area_ext<<1)+area_ext;
    wire [25:0] area5=(area_ext<<2)+area_ext;
    wire [25:0] area11=(area_ext<<3)+area3;
    wire [25:0] area75=(area5<<4)-area5;
    wire [25:0] area92=(area3<<5)-(area_ext<<2);
    wire [25:0] area88=area11<<3;
    wire [25:0] area116=(area_ext<<7)-(area3<<2);
    wire [25:0] area108=(area3<<5)+(area3<<2);
    wire [25:0] area192=area3<<6;
    wire [26:0] rect80=(rect_ext<<6)+(rect_ext<<4);
    wire [26:0] rect120=(rect_ext<<7)-(rect_ext<<3);
    wire [26:0] rect42=(rect_ext<<5)+(rect_ext<<3)+(rect_ext<<1);
    wire [26:0] rect94=(rect_ext<<7)-(rect_ext<<5)-(rect_ext<<1);
    reg candidate_ok, near_square, triangle_aspect;
    reg [1:0] shape;
    reg shape_valid;
    // Break the wide area/ratio classification from the top-nine selector.
    // Without this register, candidate -> shape_valid -> PICK_SCAN control was
    // a nine-level 9.236 ns path at a required 8.888 ns.
    reg [1:0] selected_shape;
    reg selected_shape_valid;
    always @(*) begin
        shape=0; shape_valid=0;
        if(candidate_ok) begin
            // score = 2*axis-aligned bbox area + diagonal bbox area.
            // A circle scores ~5.09*area; a rotated square 6..6.83*area.
            // Diagonal extents distinguish a diamond from an upright triangle.
            if(near_square && area128>=rect80 && area128<=rect120 &&
               score16>=area75 && score16<=area92) begin
                shape=2; shape_valid=1; // circle
            end else if(near_square && score16>=area88 && score16<=area116) begin
                shape=1; shape_valid=1; // square (including rotation)
            end else if(triangle_aspect && area128>=rect42 && area128<=rect94 &&
                        score16>=area108 && score16<=area192) begin
                shape=0; shape_valid=1; // triangle
            end
        end
        if(candidate[87:86]==0) shape_valid=0;
    end
    // Keep wide payload registers out of the 32-state control case. TD's RTL
    // expansion otherwise duplicates a feedback/hold term for every state
    // and bit (the old best_box alone produced 11,232 state-gated ANDs).
    // Each lane has one explicit write enable; the capture/publication edges
    // are exactly the same as before. No additional processing cycles.
    wire clear_candidates = state==IDLE && end_frame;
    wire store_candidate = state==PICK_WRITE && candidate[49:34]>pick_area;
    wire publish_results = state==DONE && !resume_scan;
    wire [46:0] candidate_box = {candidate[87:86],selected_shape,1'b1,
        candidate[33:25],2'b00,candidate[24:16],2'b11,
        candidate[15:8],2'b00,candidate[7:0],2'b11};
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin proc_bank<=0; work_id<=0; end
        else if(clear_candidates) begin proc_bank<=cap_bank; work_id<=frame_id; end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) result_id<=0;
        else if(publish_results) result_id<=work_id;
    end
    genvar slot;
    generate for(slot=0;slot<9;slot=slot+1) begin: candidate_lane
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin best_area[slot]<=0; best_box[slot]<=0; end
            else if(clear_candidates) begin best_area[slot]<=0; best_box[slot]<=0; end
            else if(store_candidate && pick_slot==slot) begin
                best_area[slot]<=candidate[49:34];
                best_box[slot]<=candidate_box;
            end
        end
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) boxes[slot*47 +:47]<=0;
            else if(publish_results) boxes[slot*47 +:47]<=best_box[slot];
        end
    end endgenerate
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) child<=0;
        else if(state==CHILD_SAVE) child<=stats_q;
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin candidate<=0; candidate_ok<=0; triangle_aspect<=0; end
        else if(state==EVAL_PRODUCT) begin
            candidate<=stats_q;
            candidate_ok<=parent_q==idx[9:0] && a>=MIN_AREA &&
                bw>=MIN_SIZE && bh>=MIN_SIZE && xl!=0 && yt!=0 && xr<W-1 && yb<H-1 &&
                // A full blank-row margin excludes incomplete overflow regions.
                (!overflow || ({1'b0,yb}+9'd1)<{1'b0,first_dropped_y});
            triangle_aspect<=({1'b0,bw}<<1)<=(({1'b0,bh}<<2)+bh) &&
                             ({1'b0,bh}<<1)<=(({1'b0,bw}<<2)+bw);
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) near_square<=0;
        else if(state==EVAL_PRODUCT)
            near_square<=({2'd0,bw}<<1)<=(({2'd0,bh}<<1)+bh) &&
                         ({2'd0,bh}<<1)<=(({2'd0,bw}<<1)+bw);
        else if(state==EVAL_DIAG)
            near_square<=near_square &&
                ({2'd0,du}<<1)<=(({2'd0,dv}<<1)+dv) &&
                ({2'd0,dv}<<1)<=(({2'd0,du}<<1)+du);
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) rect_area<=0;
        else if(state==EVAL_AXIS) rect_area<=product;
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) diag_area<=0;
        else if(state==EVAL_DIAG) diag_area<=product;
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin selected_shape<=0; selected_shape_valid<=0; end
        else if(state==EVAL_TEST) begin
            selected_shape<=shape;
            selected_shape_valid<=shape_valid;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;
            result_strobe<=0;
            overflow<=0; first_dropped_y<=8'hff;
            row_has_colour<=0; resume_scan<=0;
            scan_addr<=0; px<=0; py<=0; idx<=0; next_label<=1;
            left_label<=0; left_root<=0; up_root<=0; root<=0;
            current_colour<=0; left_colour<=0;
            pick_index<=0; pick_slot<=0; pick_area<=0;
        end else begin
            result_strobe<=0;
            case(state)
                IDLE: if(end_frame) begin
                    idx<=0;
                    overflow<=0; first_dropped_y<=8'hff; state<=INIT;
                    px<=0; py<=0; scan_addr<=0;
                    row_has_colour<=0; resume_scan<=0;
                end
                INIT: begin
                    if(idx==1023) begin
                        left_label<=0; left_colour<=0;
                        first_dropped_y<=8'hff;
                        next_label<=1; state<=FETCH;
                    end else idx<=idx+1'b1;
                end
                FETCH: state<=MASK;
                MASK: if(sample_colour!=0) begin
                    row_has_colour<=1;
                    current_colour<=sample_colour;
                    left_root<=left_colour==sample_colour ? left_label : 10'd0;
                    up_root<=py!=0 && line_q[11:10]==sample_colour ? line_q[9:0] : 10'd0;
                    state<=LEFT_REQ;
                end else begin current_colour<=0; root<=0; state<=ADVANCE; end
                LEFT_REQ: state<=LEFT;
                LEFT: if(left_root!=0 && parent_q!=left_root) begin
                    left_root<=parent_q; state<=LEFT_REQ;
                    end else state<=UP_REQ;
                UP_REQ: state<=UP;
                UP: if(up_root!=0 && parent_q!=up_root) begin
                    up_root<=parent_q; state<=UP_REQ;
                    end
                    else state<=SELECT;
                SELECT: begin
                    if(left_root==0 && up_root==0) begin
                        if(next_label==1024) begin
                            // Remember where allocation first failed. Components
                            // touching/later than this frontier may be incomplete.
                            if(first_dropped_y==8'hff) first_dropped_y<=py;
                            overflow<=1; root<=0; state<=ADVANCE;
                        end else begin root<=next_label[9:0]; next_label<=next_label+1'b1; state<=READ; end
                    end else begin
                        root<=left_root==0 ? up_root : up_root==0 ? left_root :
                              left_root<up_root ? left_root : up_root;
                        state<=READ;
                    end
                end
                READ: state<=WRITE;
                WRITE: state<=ADVANCE;
                ADVANCE: begin
                    left_label<=px==W-1 ? 10'd0 : root;
                    left_colour<=px==W-1 ? 2'd0 : current_colour;
                    if(scan_addr==N-1) begin
                        idx<=next_label-1'b1; resume_scan<=0; state<=MERGE;
                    end
                    else begin
                        scan_addr<=scan_addr+1'b1;
                        if(px==W-1) begin
                            px<=0; py<=py+1'b1; row_has_colour<=0;
                        end
                        else px<=px+1'b1;
                        if(px==W-1 && !row_has_colour && next_label>1) begin
                            idx<=next_label-1'b1; resume_scan<=1; state<=MERGE;
                        end else state<=FETCH;
                    end
                end
                MERGE: if(idx==0) begin idx<=1; state<=EVAL; end
                       else begin root<=idx[9:0]; state<=ROOT_REQ; end
                ROOT_REQ: state<=ROOT;
                ROOT: if(parent_q!=root) begin root<=parent_q; state<=ROOT_REQ; end
                      else if(root==idx[9:0]) state<=MERGE_NEXT;
                      else state<=CHILD_READ;
                CHILD_READ: state<=CHILD_SAVE;
                CHILD_SAVE: state<=ROOT_READ;
                ROOT_READ: state<=MERGE_WRITE;
                MERGE_WRITE: state<=MERGE_NEXT;
                MERGE_NEXT: begin idx<=idx-1'b1; state<=MERGE; end
                EVAL: if(idx>=next_label) state<=DONE; else state<=EVAL_READ;
                EVAL_READ: state<=EVAL_PRODUCT;
                EVAL_PRODUCT: state<=EVAL_MUL_WAIT;
                EVAL_MUL_WAIT: state<=EVAL_AXIS;
                EVAL_AXIS: state<=EVAL_DIAG;
                EVAL_DIAG: state<=EVAL_TEST;
                EVAL_TEST: state<=EVAL_DECIDE;
                EVAL_DECIDE: begin
                    if(selected_shape_valid) begin
                        pick_index<=1; pick_slot<=0; pick_area<=best_area[0]; state<=PICK_SCAN;
                    end else state<=EVAL_NEXT;
                end
                PICK_SCAN: begin
                    if(scanned_area<pick_area) begin
                        pick_area<=scanned_area; pick_slot<=pick_index;
                    end
                    if(pick_index==8) state<=PICK_WRITE;
                    else pick_index<=pick_index+1'b1;
                end
                PICK_WRITE: state<=EVAL_NEXT;
                EVAL_NEXT: begin idx<=idx+1'b1; state<=EVAL; end
                DONE: begin
                    if(resume_scan) begin
                        idx<=0; resume_scan<=0; state<=INIT;
                    end else begin
                        result_strobe<=1;
                        state<=IDLE;
                    end
                end
                default: state<=IDLE;
            endcase
        end
    end
endmodule

// Bound every logical primitive to one native-sized ERAM. In particular, do
// not ask TD's macro mapper to split an 88-bit table or cascade a 57,600-word
// mask. Wide tables use parallel 20x1024 slices; deep masks use 2x8192 banks.
// NOREG retains the original one-clock read. The bank index is captured on
// the same edge as the RAM address, including when CE pauses a read. Neither
// storage nor outputs are reset; existing INIT/padding/capture validity applies.
module detector_eram #(
    parameter integer DATA_WIDTH=2, ADDR_WIDTH=9, DEPTH=320,
    parameter integer SINGLE_PORT=0
)(
    input wire clk, ce, we,
    input wire [ADDR_WIDTH-1:0] waddr, raddr,
    input wire [DATA_WIDTH-1:0] wdata,
    output wire [DATA_WIDTH-1:0] q
);
    localparam integer BLOCK_WIDTH = DEPTH>1024 ? 2 : 20;
    localparam integer BLOCK_ADDR = DEPTH>1024 ? 13 : 10;
    localparam integer BLOCK_DEPTH = 2**BLOCK_ADDR;
    localparam integer WIDTH_BANKS = (DATA_WIDTH+BLOCK_WIDTH-1)/BLOCK_WIDTH;
    localparam integer DEPTH_BANKS = (DEPTH+BLOCK_DEPTH-1)/BLOCK_DEPTH;
    localparam integer PADDED_WIDTH = WIDTH_BANKS*BLOCK_WIDTH;
    localparam integer SELECT_WIDTH = ADDR_WIDTH>BLOCK_ADDR ? ADDR_WIDTH-BLOCK_ADDR : 1;
    wire [PADDED_WIDTH-1:0] padded_data = {{(PADDED_WIDTH-DATA_WIDTH){1'b0}},wdata};
    wire [BLOCK_ADDR-1:0] read_addr = raddr;
    wire [BLOCK_ADDR-1:0] write_addr = SINGLE_PORT ? raddr : waddr;
    wire [PADDED_WIDTH-1:0] bank_q [0:DEPTH_BANKS-1];
    generate
        if(DEPTH_BANKS==1) begin: one_depth_bank
            assign q=bank_q[0][DATA_WIDTH-1:0];
        end else begin: multiple_depth_banks
            reg [SELECT_WIDTH-1:0] read_bank;
            always @(posedge clk) if(ce) read_bank<=raddr>>BLOCK_ADDR;
            wire [PADDED_WIDTH-1:0] selected=bank_q[read_bank];
            assign q=selected[DATA_WIDTH-1:0];
        end
    endgenerate
    genvar d,w;
    generate for(d=0;d<DEPTH_BANKS;d=d+1) begin: depth_bank
        wire read_enable=ce && (raddr>>BLOCK_ADDR)==d;
        wire write_enable=we && ((SINGLE_PORT ? raddr : waddr)>>BLOCK_ADDR)==d;
        for(w=0;w<WIDTH_BANKS;w=w+1) begin: width_bank
            wire [BLOCK_WIDTH-1:0] qa,qb;
            assign bank_q[d][w*BLOCK_WIDTH +: BLOCK_WIDTH]=SINGLE_PORT ? qa : qb;
            PH1P_LOGIC_ERAM #(
                .DATA_WIDTH_A(BLOCK_WIDTH),.DATA_WIDTH_B(BLOCK_WIDTH),
                .ADDR_WIDTH_A(BLOCK_ADDR),.ADDR_WIDTH_B(BLOCK_ADDR),
                .DATA_DEPTH_A(BLOCK_DEPTH),.DATA_DEPTH_B(BLOCK_DEPTH),
                // PH1P DP with B write disabled is the two-address RAM.
                // SP requires ASYNC too, even with both clocks tied to clk.
                .MODE(SINGLE_PORT ? "SP" : "DP"),.CLKMODE("ASYNC"),
                .REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
                .WRITEMODE_A(SINGLE_PORT ? "READBEFOREWRITE" : "NORMAL"),.WRITEMODE_B("NORMAL"),
                .IMPLEMENT("20K(FAST)")
            ) u_ram(
                .clka(clk),.cea(SINGLE_PORT ? read_enable : write_enable),.ocea(1'b1),.rsta(1'b0),
                .wea(write_enable),.bea(1'b0),.dia(padded_data[w*BLOCK_WIDTH +: BLOCK_WIDTH]),
                .addra(write_addr),.doa(qa),
                .clkb(clk),.ceb(SINGLE_PORT ? 1'b0 : read_enable),.oceb(1'b1),.rstb(1'b0),
                .web(1'b0),.beb(1'b0),.dib({BLOCK_WIDTH{1'b0}}),.addrb(read_addr),.dob(qb),
                .ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0),.ecc_sbiterr(),.ecc_dbiterr()
            );
        end
    end endgenerate
endmodule
