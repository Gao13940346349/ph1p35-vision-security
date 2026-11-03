// Standalone zone-policy controller. Consumes descriptors, never video pixels.
// One global incident is latched, not a count of distinct people/vehicles.
// All inputs/outputs are synchronous to clk (the camera clock domain).
module security_application #(
    // Integrated build reuses the tracker's accepted, stable snapshot. The
    // standalone default retains an internal copy for independent consumers.
    parameter integer SHARED_BOXES=0,
    parameter integer CAMERA_CLK_HZ=112500000,
    parameter integer SETTLE_MS=750,
    parameter integer IMG_WIDTH=1280,
    parameter integer IMG_HEIGHT=720,
    parameter integer CONFIRM_RESULTS=3,
    parameter integer CLEAR_RESULTS=6,
    parameter integer HYSTERESIS=16,
    parameter integer MAX_SOURCE_AGE=3
)(
    input wire clk,rst_n,enable,ack,
    input wire result_strobe,result_bad,
    input wire [15:0] result_id,capture_id,
    input wire [422:0] boxes,
    output reg [2:0] app_state,severity,
    output reg [46:0] event_seed,
    output wire track_enable,
    output reg [15:0] red_count_bcd,blue_count_bcd,
    output reg [15:0] event_source_id,
    output reg status_publish
);
    localparam [2:0] HOME=0,MONITOR=1,EVENT_TRACK=2,ACK_WAIT_CLEAR=3;
    localparam integer WARN_X=(IMG_WIDTH*2)/5;
    localparam integer RESTRICT_X=(IMG_WIDTH*3)/5;
    localparam integer SETTLE_CYCLES=(CAMERA_CLK_HZ/1000)*SETTLE_MS;
    localparam integer TW=SETTLE_CYCLES<2 ? 1 : $clog2(SETTLE_CYCLES+1);
    localparam integer CW=CONFIRM_RESULTS<2 ? 1 : $clog2(CONFIRM_RESULTS+1);
    localparam integer RW=CLEAR_RESULTS<2 ? 1 : $clog2(CLEAR_RESULTS+1);
    localparam [2:0] IDLE=0,FETCH=1,GEOMETRY=2,SCORE=3,COMMIT=4;
    reg [2:0] machine;
    reg enabled_d,have_result,ack_settling;
    reg [TW-1:0] settle;
    reg [RW-1:0] clear_count;
    reg [CW-1:0] confirm_count;
    reg [15:0] last_id,work_id,boundary_id;
    reg [422:0] work_boxes;
    reg [3:0] slot;
    reg [46:0] candidate,best_box,pending_box;
    reg [2:0] best_level,pending_level;
    reg [10:0] best_x,pending_x;
    reg [9:0] pending_y;
    reg [10:0] pending_width;
    reg [9:0] pending_height;
    reg clear_threat;
    wire [15:0] age=capture_id-result_id;
    wire [15:0] order=result_id-last_id;
    wire [15:0] boundary=result_id-boundary_id;
    wire [15:0] commit_age=capture_id-work_id;
    wire fresh=result_strobe && age<=MAX_SOURCE_AGE && !boundary[15] &&
        (!have_result || (order!=0 && !order[15]));
    wire [11:0] xsum={1'b0,candidate[41:31]}+{1'b0,candidate[30:20]};
    wire [10:0] ysum={1'b0,candidate[19:10]}+{1'b0,candidate[9:0]};
    wire [10:0] cx=xsum[11:1];
    wire [9:0] cy=ysum[10:1];
    wire [10:0] width=candidate[30:20]-candidate[41:31];
    wire [9:0] height=candidate[9:0]-candidate[19:10];
    // Colour is an access grade; shape is an independent object category.
    // All nine combinations are valid application objects: red/restrict,
    // green/auth, blue/visitor x triangle/vehicle, square/equip, circle/person.
    wire usable=candidate[42] && candidate[46:45]!=0 && candidate[44:43]<=2 &&
        candidate[41:31]<candidate[30:20] && candidate[19:10]<candidate[9:0] &&
        candidate[30:20]<IMG_WIDTH && candidate[9:0]<IMG_HEIGHT;
    // Retain the boundary classification of an associated pending/active warning
    // until its centre has moved HYSTERESIS pixels back out of that zone.
    wire same_pending=candidate[46:43]==pending_box[46:43] &&
        (cx>=pending_x ? cx-pending_x : pending_x-cx)<=96 &&
        (cy>=pending_y ? cy-pending_y : pending_y-cy)<=96;
    wire restrict_zone=cx>=RESTRICT_X ||
        (same_pending && pending_level>=3 && cx>=RESTRICT_X-HYSTERESIS);
    wire warning_zone=cx>=WARN_X ||
        (same_pending && pending_level!=0 && cx>=WARN_X-HYSTERESIS);
    reg [2:0] level_q;
    reg usable_q,same_pending_q,clear_threat_q;
    reg [10:0] cx_q;
    reg [9:0] cy_q;
    reg [10:0] width_q;
    reg [9:0] height_q;
    // Geometry and policy comparisons are registered before winner selection.
    always @(posedge clk) if(machine==GEOMETRY) begin
        usable_q<=usable;cx_q<=cx;cy_q<=cy;width_q<=width;height_q<=height;
        // Authorized objects are shown but never create warning/entry events.
        // The same zone policy applies to people, equipment and vehicles.
        level_q<=(!usable || candidate[46:45]==2) ? 0 : restrict_zone ? (candidate[46:45]==1 ? 4 : 3) :
                 warning_zone ? (candidate[46:45]==1 ? 2 : 1) : 0;
        same_pending_q<=same_pending &&
            {1'b0,width}>=({1'b0,pending_width}>>1) &&
            {1'b0,width}<=({1'b0,pending_width}<<1) &&
            {1'b0,height}>=({1'b0,pending_height}>>1) &&
            {1'b0,height}<=({1'b0,pending_height}<<1);
        // After acknowledgement, conservatively wait for this incident category
        // to leave the restricted region. Slot numbers are not object identities.
        clear_threat_q<=usable && candidate[46:43]==event_seed[46:43] &&
            cx>=(severity<3 ? WARN_X-HYSTERESIS : RESTRICT_X-HYSTERESIS);
    end
    reg best_same;
    reg [9:0] best_y;
    reg [10:0] best_width;
    reg [9:0] best_height;
    assign track_enable=enable && app_state==EVENT_TRACK;
    function [15:0] increment_bcd;
        input [15:0] value;
        begin
            if(value==16'h9999) increment_bcd=value;
            else if(value[3:0]!=9) increment_bcd=value+16'h0001;
            else if(value[7:4]!=9) increment_bcd=value+16'h0007;
            else if(value[11:8]!=9) increment_bcd=value+16'h0067;
            else increment_bcd=value+16'h0667;
        end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            app_state<=HOME;severity<=0;event_seed<=0;event_source_id<=0;
            red_count_bcd<=0;blue_count_bcd<=0;status_publish<=0;
            enabled_d<=0;have_result<=0;last_id<=0;work_id<=0;boundary_id<=0;
            machine<=IDLE;settle<=0;ack_settling<=0;clear_count<=0;confirm_count<=0;
            slot<=0;candidate<=0;best_box<=0;pending_box<=0;
            if(!SHARED_BOXES) work_boxes<=0;
            best_level<=0;pending_level<=0;best_x<=0;pending_x<=0;pending_y<=0;
            pending_width<=0;pending_height<=0;clear_threat<=0;best_same<=0;
            best_y<=0;best_width<=0;best_height<=0;
        end else begin
            status_publish<=0;enabled_d<=enable;
            if(!enable) begin
                machine<=IDLE;app_state<=HOME;severity<=0;settle<=0;
                have_result<=0;confirm_count<=0;clear_count<=0;pending_level<=0;
                ack_settling<=0;
                if(enabled_d) status_publish<=1;
            end else if(!enabled_d) begin
                machine<=IDLE;app_state<=HOME;severity<=0;settle<=0;
                boundary_id<=capture_id;have_result<=0;pending_level<=0;
                confirm_count<=0;clear_count<=0;status_publish<=1;
            end else if(ack && severity!=0 && app_state!=HOME && app_state!=ACK_WAIT_CLEAR) begin
                machine<=IDLE;settle<=0;confirm_count<=0;clear_count<=0;
                boundary_id<=capture_id;have_result<=0;status_publish<=1;
                if(app_state==EVENT_TRACK) begin app_state<=ACK_WAIT_CLEAR;ack_settling<=1;end
                else begin
                    // A warning acknowledgement silences its current episode too;
                    // wait for that category to clear the warning zone before rearm.
                    app_state<=ACK_WAIT_CLEAR;ack_settling<=1;
                end
            end else if(app_state==HOME || (app_state==ACK_WAIT_CLEAR && ack_settling)) begin
                machine<=IDLE;
                if(SETTLE_CYCLES<=1 || settle>=SETTLE_CYCLES-1) begin
                    settle<=0;boundary_id<=capture_id;have_result<=0;status_publish<=1;
                    if(app_state==HOME) app_state<=MONITOR;
                    else ack_settling<=0;
                end else settle<=settle+1'b1;
            end else if(app_state==EVENT_TRACK) begin
                // The camera is moving: screen-space zones cannot define new events.
                machine<=IDLE;
            end else if(SHARED_BOXES && result_strobe && machine!=IDLE) begin
                // A newer snapshot invalidates an unfinished scan. Never mix
                // candidates from two frames or confirm a partially read one.
                machine<=IDLE;confirm_count<=0;clear_count<=0;
            end else case(machine)
                IDLE: if(fresh) begin
                    last_id<=result_id;have_result<=1;
                    if(result_bad) begin confirm_count<=0;clear_count<=0;end
                    else begin
                        work_id<=result_id;slot<=0;
                        if(!SHARED_BOXES) work_boxes<=boxes;
                        best_level<=0;best_box<=0;best_x<=0;best_same<=0;
                        clear_threat<=0;machine<=FETCH;
                    end
                end else if(result_strobe && age>MAX_SOURCE_AGE) begin
                    confirm_count<=0;clear_count<=0;
                end
                FETCH: begin
                    candidate<=SHARED_BOXES ? boxes[slot*47 +:47] : work_boxes[46:0];
                    machine<=GEOMETRY;
                end
                GEOMETRY: machine<=SCORE;
                SCORE: begin
                    if(usable_q && level_q!=0 && (level_q>best_level ||
                        (level_q==best_level && ((same_pending_q && !best_same) ||
                         (same_pending_q==best_same && cx_q<best_x))))) begin
                        best_level<=level_q;best_box<=candidate;best_x<=cx_q;
                        best_y<=cy_q;best_width<=width_q;best_height<=height_q;
                        best_same<=same_pending_q;
                    end
                    if(clear_threat_q) clear_threat<=1;
                    if(slot==8) machine<=COMMIT;
                    else begin
                        if(!SHARED_BOXES) work_boxes<=work_boxes>>47;
                        slot<=slot+1'b1;machine<=FETCH;
                    end
                end
                COMMIT: begin
                    machine<=IDLE;
                    if(commit_age>MAX_SOURCE_AGE) begin confirm_count<=0;clear_count<=0;end
                    else if(app_state==ACK_WAIT_CLEAR) begin
                        // Warning-only ACK waits for disappearance of warning, too.
                        if(clear_threat) clear_count<=0;
                        else if(CLEAR_RESULTS<=1 || clear_count>=CLEAR_RESULTS-1) begin
                            clear_count<=0;app_state<=MONITOR;severity<=0;
                            pending_level<=0;confirm_count<=0;status_publish<=1;
                        end else clear_count<=clear_count+1'b1;
                    end else if(best_level==0) begin
                        confirm_count<=0;
                        if(CLEAR_RESULTS<=1 || clear_count>=CLEAR_RESULTS-1) begin
                            clear_count<=0;
                            if(severity!=0) status_publish<=1;
                            severity<=0;pending_level<=0;
                        end else clear_count<=clear_count+1'b1;
                    end else begin
                        clear_count<=0;
                        pending_box<=best_box;pending_level<=best_level;
                        pending_x<=best_x;pending_y<=best_y;
                        pending_width<=best_width;pending_height<=best_height;
                        if(best_level==pending_level && best_same && confirm_count!=0) begin
                            if(CONFIRM_RESULTS<=1 || confirm_count>=CONFIRM_RESULTS-1) begin
                                confirm_count<=CONFIRM_RESULTS;
                                if(severity!=best_level) status_publish<=1;
                                severity<=best_level;event_seed<=best_box;event_source_id<=work_id;
                                if(best_level>=3) begin
                                    event_seed<=best_box;event_source_id<=work_id;
                                    app_state<=EVENT_TRACK;status_publish<=1;
                                    if(best_level==4) red_count_bcd<=increment_bcd(red_count_bcd);
                                    else blue_count_bcd<=increment_bcd(blue_count_bcd);
                                end
                            end else confirm_count<=confirm_count+1'b1;
                        end else begin
                            confirm_count<=1;
                            if(CONFIRM_RESULTS<=1) begin
                                severity<=best_level;event_seed<=best_box;event_source_id<=work_id;status_publish<=1;
                                if(best_level>=3) begin
                                    event_seed<=best_box;event_source_id<=work_id;
                                    app_state<=EVENT_TRACK;
                                    if(best_level==4) red_count_bcd<=increment_bcd(red_count_bcd);
                                    else blue_count_bcd<=increment_bcd(blue_count_bcd);
                                end
                            end
                        end
                    end
                end
                default: machine<=IDLE;
            endcase
        end
    end
endmodule
