// Side-channel detector. No ready/backpressure and no changes to the RGB stream.
// 4-connected components on a 1/4-resolution binary mask; largest round blob wins.
// Inputs: four RGB888 pixels per valid beat, P0 at [95:72], early SOF allowed.
module red_circle_detector #(
    parameter integer IMG_WIDTH = 1280,
    parameter integer IMG_HEIGHT = 720,
    parameter integer MIN_AREA = 40,
    parameter integer MIN_SIZE = 8
)(
    input wire clk, rst_n, enable, sof, valid,
    input wire [95:0] rgb,
    output reg result_strobe,
    output reg [15:0] result_id,
    output reg box_valid,
    output reg [10:0] xmin, xmax,
    output reg [9:0] ymin, ymax,
    output wire busy,
    output reg overflow
);
    localparam W = IMG_WIDTH/4, H = IMG_HEIGHT/4, N = W*H;
    localparam IDLE=0, INIT=1, FETCH=2, MASK=3, LEFT=4, UP=5,
        SELECT=6, READ=7, WRITE=8, ADVANCE=9, MERGE=10,
        ROOT=11, CHILD_READ=12, CHILD_SAVE=13, ROOT_READ=14,
        MERGE_WRITE=15, MERGE_NEXT=16, EVAL=17, EVAL_READ=18,
        EVAL_PRODUCT=19, EVAL_TEST=20, EVAL_NEXT=21, DONE=22;
    reg [4:0] state;
    assign busy = state != IDLE;

    function is_red;
        input [23:0] p;
        reg [10:0] r,g,b;
        begin
            r={3'd0,p[23:16]}; g={3'd0,p[15:8]}; b={3'd0,p[7:0]};
            is_red = r>=48 && r>=g+20 && r>=b+20 &&
                     (r<<2)>=(g<<2)+g && (r<<2)>=(b<<2)+b;
        end
    endfunction
    reg [3:0] red_bits;
    wire [2:0] votes = {2'd0,red_bits[3]}+{2'd0,red_bits[2]}+
                       {2'd0,red_bits[1]}+{2'd0,red_bits[0]};
    (* ram_style = "block" *) reg mask0 [0:N-1];
    (* ram_style = "block" *) reg mask1 [0:N-1];
    reg cap_bank, proc_bank, capturing;
    reg [8:0] cx;
    reg [9:0] cy;
    reg [15:0] cap_addr, frame_id, work_id;
    reg sof_d;
    wire start_frame = sof && !sof_d;
    wire end_frame = capturing && valid && cx==W-1 && cy==IMG_HEIGHT-1;
    wire capture_write = capturing && valid && cy[1:0]==2 && !start_frame;
    reg capture_we_d, cap_bank_d;
    reg [15:0] cap_addr_d;
    // Separate RGB classification from horizontal voting/RAM write timing.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin red_bits<=0; capture_we_d<=0; cap_bank_d<=0; cap_addr_d<=0; end
        else begin
            red_bits<={is_red(rgb[95:72]),is_red(rgb[71:48]),is_red(rgb[47:24]),is_red(rgb[23:0])};
            capture_we_d<=capture_write; cap_bank_d<=cap_bank; cap_addr_d<=cap_addr;
        end
    end
    reg [15:0] scan_addr;
    reg mask_q0, mask_q1;
    // Synchronous read, no RAM reset/clear: every captured mask cell is overwritten.
    always @(posedge clk) begin
        if(capture_we_d && !cap_bank_d) mask0[cap_addr_d] <= votes>=2;
        if(capture_we_d &&  cap_bank_d) mask1[cap_addr_d] <= votes>=2;
        mask_q0 <= mask0[scan_addr];
        mask_q1 <= mask1[scan_addr];
    end
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

    // Bounded union-find label table. Label 0 means background. More than 127
    // provisional regions invalidates this frame rather than merging unrelated blobs.
    reg [6:0] parent [0:127];
    (* ram_style = "block" *) reg [6:0] line_label [0:W-1];
    reg [6:0] line_q, left_label, left_root, up_root, root;
    reg [7:0] next_label, idx;
    reg [8:0] px;
    reg [7:0] py;
    (* ram_style = "block" *) reg [49:0] stats [0:127];
    // Packed statistics: area16, xmin9, xmax9, ymin8, ymax8.
    reg [49:0] stats_q, child;
    wire [6:0] stats_addr = (state==INIT || state==CHILD_READ ||
        state==CHILD_SAVE || state==EVAL || state==EVAL_READ) ? idx[6:0] : root;
    wire [15:0] a = stats_q[49:34];
    wire [8:0] xl = stats_q[33:25], xr = stats_q[24:16];
    wire [7:0] yt = stats_q[15:8], yb = stats_q[7:0];
    wire [15:0] merged_a = a + child[49:34];
    wire [8:0] merged_xl = a==0 || child[33:25]<xl ? child[33:25] : xl;
    wire [8:0] merged_xr = a==0 || child[24:16]>xr ? child[24:16] : xr;
    wire [7:0] merged_yt = a==0 || child[15:8]<yt ? child[15:8] : yt;
    wire [7:0] merged_yb = a==0 || child[7:0]>yb ? child[7:0] : yb;
    wire stats_we = state==INIT || state==WRITE || (state==MERGE_WRITE && child[49:34]!=0);
    wire [6:0] stats_waddr = state==INIT ? idx[6:0] : root;
    wire [49:0] stats_wdata = state==INIT ? 50'd0 : state==WRITE ?
        {a+16'd1, (a==0 || px<xl ? px : xl), (a==0 || px>xr ? px : xr),
         (a==0 || py<yt ? py : yt), (a==0 || py>yb ? py : yb)} :
        {merged_a,merged_xl,merged_xr,merged_yt,merged_yb};
    always @(posedge clk) begin
        line_q <= line_label[px];
        if(state==ADVANCE) line_label[px] <= root;
        stats_q <= stats[stats_addr];
        if(stats_we) stats[stats_waddr] <= stats_wdata;
    end
    reg [15:0] best_area;
    reg [8:0] best_xl,best_xr;
    reg [7:0] best_yt,best_yb;
    reg [17:0] rect_area;
    reg [49:0] candidate;
    wire [9:0] bw = {1'b0,xr}-{1'b0,xl}+10'd1;
    wire [8:0] bh = {1'b0,yb}-{1'b0,yt}+9'd1;
    wire [23:0] area128 = {1'b0,candidate[49:34],7'd0};
    wire [23:0] rect88 = ({6'd0,rect_area}<<6)+({6'd0,rect_area}<<4)+({6'd0,rect_area}<<3);
    wire [23:0] rect114 = ({6'd0,rect_area}<<6)+({6'd0,rect_area}<<5)+
                         ({6'd0,rect_area}<<4)+({6'd0,rect_area}<<1);
    reg candidate_ok;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE; proc_bank<=0; work_id<=0; result_id<=0;
            result_strobe<=0; box_valid<=0; xmin<=0; xmax<=0; ymin<=0; ymax<=0;
            overflow<=0; scan_addr<=0; px<=0; py<=0; idx<=0; next_label<=1;
            left_label<=0; left_root<=0; up_root<=0; root<=0; child<=0;
            best_area<=0; best_xl<=0; best_xr<=0; best_yt<=0; best_yb<=0;
            rect_area<=0; candidate<=0; candidate_ok<=0;
        end else begin
            result_strobe<=0;
            case(state)
                IDLE: if(end_frame) begin
                    proc_bank<=cap_bank; work_id<=frame_id; idx<=0;
                    best_area<=0; overflow<=0; state<=INIT;
                end
                INIT: begin
                    parent[idx[6:0]]<=idx[6:0];
                    if(idx==127) begin
                        px<=0; py<=0; scan_addr<=0; left_label<=0;
                        next_label<=1; state<=FETCH;
                    end else idx<=idx+1'b1;
                end
                FETCH: state<=MASK;
                MASK: if(proc_bank ? mask_q1 : mask_q0) begin
                    left_root<=left_label; up_root<=py==0 ? 7'd0 : line_q;
                    state<=LEFT;
                end else begin root<=0; state<=ADVANCE; end
                LEFT: if(left_root!=0 && parent[left_root]!=left_root)
                    left_root<=parent[left_root];
                    else state<=UP;
                UP: if(up_root!=0 && parent[up_root]!=up_root)
                    up_root<=parent[up_root];
                    else state<=SELECT;
                SELECT: begin
                    if(left_root==0 && up_root==0) begin
                        if(next_label==128) begin overflow<=1; state<=DONE; end
                        else begin root<=next_label[6:0]; next_label<=next_label+1'b1; state<=READ; end
                    end else begin
                        root<=left_root==0 ? up_root : up_root==0 ? left_root :
                              left_root<up_root ? left_root : up_root;
                        if(left_root!=0 && up_root!=0 && left_root!=up_root)
                            if(left_root<up_root) parent[up_root]<=left_root;
                            else parent[left_root]<=up_root;
                        state<=READ;
                    end
                end
                READ: state<=WRITE;
                WRITE: state<=ADVANCE;
                ADVANCE: begin
                    left_label<=px==W-1 ? 7'd0 : root;
                    if(scan_addr==N-1) begin idx<=next_label-1'b1; state<=MERGE; end
                    else begin
                        scan_addr<=scan_addr+1'b1;
                        if(px==W-1) begin px<=0; py<=py+1'b1; end
                        else px<=px+1'b1;
                        state<=FETCH;
                    end
                end
                MERGE: if(idx==0) begin idx<=1; state<=EVAL; end
                       else begin root<=idx[6:0]; state<=ROOT; end
                ROOT: if(parent[root]!=root) root<=parent[root];
                      else if(root==idx[6:0]) state<=MERGE_NEXT;
                      else state<=CHILD_READ;
                CHILD_READ: state<=CHILD_SAVE;
                CHILD_SAVE: begin child<=stats_q; state<=ROOT_READ; end
                ROOT_READ: state<=MERGE_WRITE;
                MERGE_WRITE: state<=MERGE_NEXT;
                MERGE_NEXT: begin idx<=idx-1'b1; state<=MERGE; end
                EVAL: if(idx>=next_label) state<=DONE; else state<=EVAL_READ;
                EVAL_READ: state<=EVAL_PRODUCT;
                EVAL_PRODUCT: begin
                    candidate<=stats_q;
                    rect_area<=bw*bh;
                    candidate_ok<=parent[idx[6:0]]==idx[6:0] && a>=MIN_AREA &&
                        bw>=MIN_SIZE && bh>=MIN_SIZE && xl!=0 && yt!=0 && xr<W-1 && yb<H-1 &&
                        ({3'd0,bw}<<2)<=(({4'd0,bh}<<2)+bh) &&
                        ({4'd0,bh}<<2)<=(({3'd0,bw}<<2)+bw);
                    state<=EVAL_TEST;
                end
                EVAL_TEST: begin
                    // Filled circle pi/4 ~= 0.785. Reject solid squares and thin rings.
                    if(candidate_ok && area128>=rect88 && area128<=rect114 &&
                       candidate[49:34]>best_area) begin
                        best_area<=candidate[49:34]; best_xl<=candidate[33:25];
                        best_xr<=candidate[24:16]; best_yt<=candidate[15:8]; best_yb<=candidate[7:0];
                    end
                    state<=EVAL_NEXT;
                end
                EVAL_NEXT: begin idx<=idx+1'b1; state<=EVAL; end
                DONE: begin
                    result_id<=work_id; result_strobe<=1;
                    box_valid<=best_area!=0 && !overflow;
                    xmin<={best_xl,2'b00}; xmax<={best_xr,2'b11};
                    ymin<={best_yt,2'b00}; ymax<={best_yb,2'b11};
                    state<=IDLE;
                end
                default: state<=IDLE;
            endcase
        end
    end
endmodule
