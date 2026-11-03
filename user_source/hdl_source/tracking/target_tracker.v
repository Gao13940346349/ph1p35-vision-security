// Nine-box, single-target association. Runs in the detector/camera clock domain.
// No video buffer or arithmetic multiplier is used. A result takes at most 77
// clocks (under 0.7 us at 112.5 MHz); detector results are thousands of clocks apart.
// Geometry, absolute deltas and score sums are separated by registers so the
// final best-candidate comparison never includes coordinate arithmetic.
module target_tracker #(
    parameter integer SECURITY_SUPPORT=0,
    parameter integer IMG_WIDTH=1280,
    parameter integer IMG_HEIGHT=720,
    parameter integer CONFIRM_RESULTS=3,
    parameter integer MAX_SOURCE_AGE=3,
    parameter integer LOST_HOLD_FRAMES=15,
    parameter integer SHAPE_GRACE_RESULTS=2,
    parameter [1:0] PREFERRED_COLOUR=2'd1,
    parameter [1:0] PREFERRED_SHAPE=2'd2
)(
    input wire clk, rst_n, enable, switch_target,
    input wire result_strobe, result_bad,
    input wire [15:0] result_id, capture_id,
    input wire [422:0] boxes,
    // Camera-domain application override. Ignored when SECURITY_SUPPORT=0.
    input wire app_mode,app_track_enable,
    input wire [46:0] app_seed,
    output reg publish,
    // Only actual detector results may be associated with a DDR source frame.
    // Enable/switch invalidations still use publish for the control mailbox.
    output reg result_publish,
    // Camera-domain accepted snapshot, shared with the application policy.
    // publish_boxes is immutable until the next snapshot_strobe.
    output reg snapshot_strobe,snapshot_bad,
    output wire [15:0] snapshot_id,
    output reg [15:0] source_id, source_age,
    output reg [422:0] publish_boxes,
    // 47-bit layout matches color_shape_detector. Colour/shape stay equal to
    // the acquisition category during short shape-classification glitches.
    output reg [46:0] selected_box,
    output reg [7:0] track_id, session_id,
    // 0 OFF, 1 ACQUIRE, 2 LOCKED, 3 LOST. Only LOCKED can move the servo.
    output reg [1:0] track_state,
    output reg measurement_valid
);
    localparam [1:0] OFF=0, ACQUIRE=1, LOCKED=2, LOST=3;
    localparam [3:0] IDLE=0, FETCH=1, SCORE=2, CYCLE_FETCH=3,
                     CYCLE_SCORE=4, COMMIT=5, GEOMETRY=6, DELTAS=7, METRICS=8,
                     ADMIT=9;
    reg [3:0] machine;
    reg cycle_scan;
    wire application=SECURITY_SUPPORT ? app_mode : 1'b0;
    wire application_tracking=SECURITY_SUPPORT ? app_track_enable : 1'b0;
    reg app_mode_d,app_tracking_d;
    reg enabled_d, switch_pending, have_result, have_reference, locked_identity;
    reg [15:0] boundary_id, last_result_id, last_match_id;
    reg [15:0] work_id;
    assign snapshot_id=work_id;
    reg [3:0] index, match_slot;
    reg [7:0] confirmations, shape_misses;
    reg work_switch, best_match_valid, best_acquire_valid, best_preferred;
    reg next_valid, left_valid;
    reg [46:0] candidate, reference_box, best_match, best_acquire, best_next, best_left;
    reg [13:0] best_match_score, best_acquire_score;
    reg [11:0] best_acquire_size;
    reg [20:0] cycle_anchor, next_key, left_key;

    wire [15:0] incoming_age=capture_id-result_id;
    wire [15:0] incoming_order=result_id-last_result_id;
    wire [15:0] incoming_boundary=result_id-boundary_id;
    wire [15:0] incoming_match_gap=result_id-last_match_id;
    wire [15:0] commit_age=capture_id-work_id;
    wire accepted = result_strobe && incoming_age<=MAX_SOURCE_AGE &&
        !incoming_boundary[15] && (!have_result ||
        (incoming_order!=0 && !incoming_order[15]));

    // Header validation contains circular frame-ID subtraction/comparison. Its
    // result must not directly drive the 423-bit public snapshot's clock enable.
    // Capture the complete one-cycle input event with only result_strobe as CE,
    // then publish it atomically in ADMIT using the registered verdict. The
    // public snapshot stays unchanged for rejected frames and while scanning.
    // These registers need no reset: IDLE must capture an event before ADMIT.
    reg [422:0] incoming_boxes_q;
    reg [15:0] incoming_id_q, incoming_age_q;
    reg incoming_bad_q, accepted_q, incoming_expired_q;
    always @(posedge clk) begin
        if(result_strobe) begin
            incoming_boxes_q<=boxes;
            incoming_id_q<=result_id;
            incoming_age_q<=incoming_age;
            incoming_bad_q<=result_bad;
            accepted_q<=accepted;
            incoming_expired_q<=incoming_match_gap>LOST_HOLD_FRAMES;
        end
    end
    wire [1:0] colour=candidate[46:45], shape=candidate[44:43];
    wire [10:0] xmin=candidate[41:31], xmax=candidate[30:20];
    wire [9:0] ymin=candidate[19:10], ymax=candidate[9:0];
    wire [11:0] xsum={1'b0,xmin}+{1'b0,xmax};
    wire [10:0] ysum={1'b0,ymin}+{1'b0,ymax};
    wire [10:0] cx=xsum[11:1], width=xmax-xmin;
    wire [9:0] cy=ysum[10:1], height=ymax-ymin;
    wire usable=candidate[42] && colour!=0 && shape!=3 &&
        xmin<xmax && ymin<ymax && xmax<IMG_WIDTH && ymax<IMG_HEIGHT;
    wire preferred=colour==PREFERRED_COLOUR && shape==PREFERRED_SHAPE;
    wire [11:0] ref_xsum={1'b0,reference_box[41:31]}+{1'b0,reference_box[30:20]};
    wire [10:0] ref_ysum={1'b0,reference_box[19:10]}+{1'b0,reference_box[9:0]};
    wire [10:0] ref_x=ref_xsum[11:1], ref_width=reference_box[30:20]-reference_box[41:31];
    wire [9:0] ref_y=ref_ysum[10:1], ref_height=reference_box[9:0]-reference_box[19:10];
    wire [15:0] match_gap=work_id-last_match_id;
    wire [8:0] match_limit=match_gap>=8 ? 9'd192 : 9'd64+{match_gap[3:0],4'd0};

    // The candidate and reference remain stable throughout each scan slot.
    // These datapath registers need no reset: only SCORE/CYCLE_SCORE consume
    // them, after the corresponding stages have overwritten every field.
    reg usable_q, preferred_q, category_q, size_ok_q;
    reg [10:0] cx_q, width_q, ref_x_q, ref_width_q;
    reg [9:0] cy_q, height_q, ref_y_q, ref_height_q;
    reg [10:0] dx_q, dw_q, center_dx_q;
    reg [9:0] dy_q, dh_q, center_dy_q;
    reg [8:0] match_limit_q;
    reg [11:0] distance_q, size_sum_q;
    reg [13:0] match_score_q, acquire_score_q;
    reg [20:0] candidate_key_q;
    wire [11:0] delta_position={1'b0,dx_q}+{2'b0,dy_q};
    wire [11:0] delta_size={1'b0,dw_q}+{2'b0,dh_q};
    wire match_ok=usable_q && category_q && size_ok_q && distance_q<=match_limit_q;

    always @(posedge clk) begin
        if(machine==GEOMETRY) begin
            usable_q<=usable; preferred_q<=preferred;
            category_q<=colour==reference_box[46:45] &&
                (shape==reference_box[44:43] ||
                 (locked_identity && shape_misses<SHAPE_GRACE_RESULTS));
            cx_q<=cx; cy_q<=cy; width_q<=width; height_q<=height;
            ref_x_q<=ref_x; ref_y_q<=ref_y;
            ref_width_q<=ref_width; ref_height_q<=ref_height;
            match_limit_q<=match_limit;
            candidate_key_q<={cx,cy};
        end
        if(machine==DELTAS) begin
            dx_q<=cx_q>ref_x_q ? cx_q-ref_x_q : ref_x_q-cx_q;
            dy_q<=cy_q>ref_y_q ? cy_q-ref_y_q : ref_y_q-cy_q;
            dw_q<=width_q>ref_width_q ? width_q-ref_width_q : ref_width_q-width_q;
            dh_q<=height_q>ref_height_q ? height_q-ref_height_q : ref_height_q-height_q;
            center_dx_q<=cx_q>IMG_WIDTH/2 ? cx_q-IMG_WIDTH/2 : IMG_WIDTH/2-cx_q;
            center_dy_q<=cy_q>IMG_HEIGHT/2 ? cy_q-IMG_HEIGHT/2 : IMG_HEIGHT/2-cy_q;
            size_ok_q<={1'b0,width_q}>=({1'b0,ref_width_q}>>1) &&
                {1'b0,width_q}<=({1'b0,ref_width_q}<<1) &&
                {1'b0,height_q}>=({1'b0,ref_height_q}>>1) &&
                {1'b0,height_q}<=({1'b0,ref_height_q}<<1);
        end
        if(machine==METRICS) begin
            distance_q<=delta_position;
            match_score_q<={2'b0,delta_position}+{2'b0,delta_size};
            acquire_score_q<={3'b0,center_dx_q}+{4'b0,center_dy_q};
            size_sum_q<={1'b0,width_q}+{2'b0,height_q};
        end
    end
    wire [11:0] matched_xsum={1'b0,best_match[41:31]}+{1'b0,best_match[30:20]};
    wire [10:0] matched_ysum={1'b0,best_match[19:10]}+{1'b0,best_match[9:0]};

    // Explicit switching creates a new identity. The selected category is held
    // for association after that, so other larger objects cannot steal it.
    task acquire_new;
        input [46:0] box;
        begin
            reference_box<=box; selected_box<=box; have_reference<=1;
            locked_identity<=(CONFIRM_RESULTS<=1);
            confirmations<=1; shape_misses<=0; last_match_id<=work_id;
            track_id<=track_id==8'hff ? 8'd1 : track_id+1'b1;
            track_state<=(CONFIRM_RESULTS<=1) ? LOCKED : ACQUIRE;
            measurement_valid<=(CONFIRM_RESULTS<=1);
        end
    endtask

    task retain_match;
        begin
            reference_box<={reference_box[46:43],best_match[42:0]};
            selected_box<={reference_box[46:43],best_match[42:0]};
            last_match_id<=work_id;
            if(best_match[44:43]==reference_box[44:43]) shape_misses<=0;
            else shape_misses<=shape_misses+1'b1;
            if(locked_identity || confirmations>=CONFIRM_RESULTS-1) begin
                locked_identity<=1; track_state<=LOCKED; measurement_valid<=1;
            end else begin
                confirmations<=confirmations+1'b1;
                track_state<=ACQUIRE; measurement_valid<=0;
            end
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            app_mode_d<=0;app_tracking_d<=0;
            machine<=IDLE; cycle_scan<=0; enabled_d<=0; switch_pending<=0; have_result<=0;
            have_reference<=0; locked_identity<=0; boundary_id<=0;
            last_result_id<=0; last_match_id<=0; work_id<=0;
            source_id<=0; source_age<=0; publish_boxes<=0; publish<=0; result_publish<=0;
            snapshot_strobe<=0;snapshot_bad<=0;
            selected_box<=0; track_id<=0; session_id<=0; track_state<=OFF;
            measurement_valid<=0; confirmations<=0; shape_misses<=0;
            index<=0; match_slot<=0; candidate<=0; reference_box<=0;
            best_match<=0; best_acquire<=0; best_next<=0; best_left<=0;
            best_match_score<=0; best_acquire_score<=0; best_acquire_size<=0;
            cycle_anchor<=0; next_key<=0; left_key<=0; work_switch<=0;
            best_match_valid<=0; best_acquire_valid<=0; best_preferred<=0;
            next_valid<=0; left_valid<=0;
        end else begin
            enabled_d<=enable; publish<=0; result_publish<=0;snapshot_strobe<=0;
            app_mode_d<=application;app_tracking_d<=application_tracking;
            if(!enable) begin
                machine<=IDLE; switch_pending<=0; have_result<=0;
                have_reference<=0; locked_identity<=0; confirmations<=0;
                shape_misses<=0; track_state<=OFF; measurement_valid<=0;
                selected_box<=0; reference_box<=0;
                if(enabled_d) publish<=1;
            end else if(!enabled_d) begin
                machine<=IDLE; boundary_id<=capture_id;
                session_id<=session_id+1'b1;
                track_state<=ACQUIRE; measurement_valid<=0; publish<=1;
            end else if(application!=app_mode_d ||
                (application && application_tracking!=app_tracking_d)) begin
                // Invalidate in-flight ordinary acquisitions on entering an application
                // and seed exactly the event selected by the application controller.
                machine<=IDLE;switch_pending<=0;have_result<=0;boundary_id<=capture_id;
                session_id<=session_id+1'b1;measurement_valid<=0;publish<=1;
                if(application && application_tracking) begin
                    reference_box<=app_seed;selected_box<=app_seed;have_reference<=1;
                    locked_identity<=1;confirmations<=CONFIRM_RESULTS;shape_misses<=0;
                    last_match_id<=capture_id;track_state<=ACQUIRE;
                    track_id<=track_id==8'hff ? 8'd1 : track_id+1'b1;
                end else begin
                    have_reference<=0;locked_identity<=0;confirmations<=0;shape_misses<=0;
                    selected_box<=0;reference_box<=0;track_state<=application ? OFF : ACQUIRE;
                end
            end else if(switch_target && !application) begin
                // Abort an older result. Await a source frame captured after
                // this user action before confirming/moving a new target.
                machine<=IDLE; switch_pending<=1; boundary_id<=capture_id;
                session_id<=session_id+1'b1;
                track_state<=ACQUIRE; measurement_valid<=0; publish<=1;
            end else case(machine)
                IDLE: if(result_strobe) machine<=ADMIT;
                ADMIT: begin
                    machine<=IDLE;
                    if(accepted_q) begin
                        last_result_id<=incoming_id_q; have_result<=1;
                        work_id<=incoming_id_q;
                        publish_boxes<=incoming_boxes_q;
                        snapshot_strobe<=1;snapshot_bad<=incoming_bad_q;
                        best_match_valid<=0; best_acquire_valid<=0;
                        best_preferred<=0; next_valid<=0; left_valid<=0;
                        work_switch<=switch_pending; index<=0;
                        if(incoming_bad_q) begin
                            source_id<=incoming_id_q; source_age<=incoming_age_q;
                            measurement_valid<=0; publish<=1; result_publish<=1;
                            track_state<=have_reference ? LOST : ACQUIRE;
                            if(!locked_identity) confirmations<=0;
                            if(!application && have_reference && incoming_expired_q) begin
                                have_reference<=0; locked_identity<=0;
                                selected_box<=0; track_state<=ACQUIRE; confirmations<=0;
                            end
                        end else machine<=FETCH;
                    end
                end
                FETCH: begin
                    candidate<=publish_boxes[index*47 +:47]; cycle_scan<=0; machine<=GEOMETRY;
                end
                GEOMETRY: machine<=cycle_scan ? CYCLE_SCORE : DELTAS;
                DELTAS: machine<=METRICS;
                METRICS: machine<=SCORE;
                SCORE: begin
                    if(usable_q && (!best_acquire_valid || (preferred_q && !best_preferred) ||
                        (preferred_q==best_preferred && (acquire_score_q<best_acquire_score ||
                        (acquire_score_q==best_acquire_score && size_sum_q>best_acquire_size))))) begin
                        best_acquire<=candidate; best_acquire_valid<=1;
                        best_preferred<=preferred_q; best_acquire_score<=acquire_score_q;
                        best_acquire_size<=size_sum_q;
                    end
                    if(have_reference && match_ok && (!best_match_valid || match_score_q<best_match_score)) begin
                        best_match<=candidate; best_match_valid<=1;
                        best_match_score<=match_score_q; match_slot<=index;
                    end
                    if(index==8) machine<=COMMIT;
                    else begin index<=index+1'b1; machine<=FETCH; end
                end
                CYCLE_FETCH: begin
                    candidate<=publish_boxes[index*47 +:47]; cycle_scan<=1; machine<=GEOMETRY;
                end
                CYCLE_SCORE: begin
                    if(usable_q && !(best_match_valid && index==match_slot)) begin
                        if(!left_valid || candidate_key_q<left_key) begin
                            best_left<=candidate; left_key<=candidate_key_q; left_valid<=1;
                        end
                        if(candidate_key_q>cycle_anchor && (!next_valid || candidate_key_q<next_key)) begin
                            best_next<=candidate; next_key<=candidate_key_q; next_valid<=1;
                        end
                    end
                    if(index==8) begin work_switch<=0; machine<=COMMIT; end
                    else begin index<=index+1'b1; machine<=CYCLE_FETCH; end
                end
                COMMIT: begin
                    if(work_switch) begin
                        cycle_anchor<=best_match_valid ? {matched_xsum[11:1],matched_ysum[10:1]} :
                            (have_reference ? {ref_x,ref_y} : 21'd0);
                        index<=0; machine<=CYCLE_FETCH;
                    end else begin
                        source_id<=work_id; source_age<=commit_age; publish<=1; result_publish<=1;
                        measurement_valid<=0; machine<=IDLE;
                        if(commit_age>MAX_SOURCE_AGE) begin
                            // A new SOF may cross the bounded scan. Preserve
                            // the identity, but never control from aged data.
                            track_state<=have_reference ? LOST : ACQUIRE;
                            if(!locked_identity) confirmations<=0;
                        end else if(application) begin
                            // Monitoring publishes detections without steering. Event tracking
                            // associates only with the seeded category/reference, never falls
                            // back to generic acquisition or switches to an authorized target.
                            if(!application_tracking) begin
                                selected_box<=0;have_reference<=0;locked_identity<=0;
                                track_state<=OFF;confirmations<=0;
                            end else if(have_reference && best_match_valid) retain_match();
                            else track_state<=LOST;
                        end else if(switch_pending) begin
                            switch_pending<=0;
                            if(next_valid) acquire_new(best_next);
                            else if(left_valid) acquire_new(best_left);
                            else if(best_match_valid) retain_match();
                            else begin
                                have_reference<=0; locked_identity<=0; selected_box<=0;
                                track_state<=ACQUIRE; confirmations<=0;
                            end
                        end else if(have_reference && best_match_valid && match_gap<=LOST_HOLD_FRAMES) begin
                            retain_match();
                        end else if(have_reference && match_gap<=LOST_HOLD_FRAMES) begin
                            track_state<=LOST; confirmations<=0;
                        end else if(best_acquire_valid) begin
                            acquire_new(best_acquire);
                        end else begin
                            have_reference<=0; locked_identity<=0; selected_box<=0;
                            track_state<=ACQUIRE; confirmations<=0;
                        end
                    end
                end
                default: machine<=IDLE;
            endcase
        end
    end
endmodule
