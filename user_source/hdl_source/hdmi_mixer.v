module hdmi_mixer #(
    parameter H_OFFSET = 128,
    parameter V_OFFSET = 60,
    parameter IMG_WIDTH = 1024,
    parameter IMG_HEIGHT = 600,
    parameter integer DEBUG_MODE = 0,
    parameter integer DETECT_SUPPORT = 0,
    parameter integer TRACKING_SUPPORT = 0,
    parameter integer SECURITY_SUPPORT = 0
)(
    input wire        I_clk,
    input wire        I_rst_n,

    input wire        I_video_vsync,
    input wire        I_video_hsync,
    input wire        I_video_de,
    input wire        I_video_user,
    input wire        I_video_last,

    input wire[3:0]   I_debug_status,

    input wire        I_cal_enable,
    input wire        I_notice_detail_enable,
    input wire        I_notice_detect_enable,
    input wire [422:0] I_detect_box, // Nine independent colour/shape/box descriptors
    input wire [46:0] I_track_box,
    input wire [7:0] I_track_id,
    input wire [1:0] I_track_state,
    input wire I_security_enable,
    // {state3, severity3, red BCD16, blue BCD16, event source16}.
    input wire [53:0] I_security_status,
    input wire [60:0] I_frame_statistics, // valid, DET, HDMI, REP, NEW, CAM (BCD)
    input wire        I_latency_valid,
    input wire        I_latency_overflow,
    input wire [3:0]  I_latency_frames,
    input wire        I_cam_cfg_error,
    input wire        I_cam_cfg_ready,
    input wire[2:0]    I_cal_mode,
    input wire[3:0]    I_cal_path_status,
    input wire[7:0]    I_link_status,
    input wire        I_cal_awb_enable,
    input wire        I_cal_ae_enable,
    input wire[19:0]  I_cal_raw_b_bcd,
    input wire[19:0]  I_cal_raw_gb_bcd,
    input wire[19:0]  I_cal_raw_gr_bcd,
    input wire[19:0]  I_cal_raw_r_bcd,
    input wire[19:0]  I_cal_rgb_r_bcd,
    input wire[19:0]  I_cal_rgb_g_bcd,
    input wire[19:0]  I_cal_rgb_b_bcd,
    input wire[19:0]  I_cal_clip_bcd,
    input wire[19:0]  I_cal_exposure_bcd,
    input wire[19:0]  I_cal_gain_bcd,
    input wire[19:0] I_cal_dgain_bcd,
    input wire I_cal_params_valid,
    input wire [26:0] I_cal_param_levels, // {DG,AG,EXP}, 9-bit lengths 0..320

    output wire       O_video_rd_en,
    input wire[23:0]  I_video_rd_data,

    output reg        O_hdmi_vsync,
    output reg        O_hdmi_hsync,
    output reg        O_hdmi_de,
    output reg[23:0]  O_hdmi_data
);

    reg        S_video_vsync_1d;
    reg        S_video_vsync_2d;
    reg        S_video_hsync_1d;
    reg        S_video_hsync_2d;
    reg        S_video_de_1d;
    reg        S_video_de_2d;
    reg [11:0] S_x;
    reg [11:0] S_y;
    reg [11:0] S_x_1d;
    reg [11:0] S_x_2d;
    reg [11:0] S_y_1d;
    reg [11:0] S_y_2d;

    // The frame marker accompanies pixel (0,0). Use its coordinates immediately,
    // before the counter registers update, so full-screen video never loses it.
    wire [11:0] S_pixel_x = I_video_user ? 12'd0 : S_x;
    wire [11:0] S_pixel_y = I_video_user ? 12'd0 : S_y;
    wire S_in_image = I_video_de &&
                      (S_pixel_x >= H_OFFSET) && (S_pixel_x < H_OFFSET + IMG_WIDTH) &&
                      (S_pixel_y >= V_OFFSET) && (S_pixel_y < V_OFFSET + IMG_HEIGHT);
    reg S_in_image_1d,S_in_image_2d;
    reg S_logo_region_1,S_logo_region_2,S_notice_region_1,S_notice_region_2;
    reg S_roi_border_1,S_roi_border_2;
    wire S_scopes_hit;
    wire [23:0] S_scopes_colour;
    video_scopes #(.H_OFFSET(H_OFFSET),.V_OFFSET(V_OFFSET),
        .IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) u_scopes(
        .clk(I_clk),.rst_n(I_rst_n),.enable(I_cal_enable),
        .sample_de(S_in_image_2d),.sample_x(S_x_2d),.sample_y(S_y_2d),.rgb(I_video_rd_data),
        .draw_de(I_video_de),.draw_x(S_pixel_x),.draw_y(S_pixel_y),
        .hit(S_scopes_hit),.colour(S_scopes_colour));
    // Three narrow parameter cards below the general statistics. All scaling
    // runs once per source-frame event in camera_parameter_levels, not here.
    reg S_param_hit_1,S_param_hit_2,S_param_valid_1;
    reg [8:0] S_param_x_1,S_param_level_1;
    reg [5:0] S_param_y_1;
    reg [23:0] S_param_colour_2;
    wire [8:0] S_param_knob = S_param_level_1>314 ? 9'd314 : S_param_level_1;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_param_hit_1<=0;S_param_hit_2<=0;S_param_valid_1<=0;
            S_param_x_1<=0;S_param_y_1<=0;S_param_level_1<=0;
            S_param_colour_2<=0;
        end else begin
            S_param_hit_1<=I_cal_enable && I_video_de &&
                S_pixel_x>=8 && S_pixel_x<344 &&
                ((S_pixel_y>=464 && S_pixel_y<512) ||
                 (S_pixel_y>=520 && S_pixel_y<568) ||
                 (S_pixel_y>=576 && S_pixel_y<624));
            S_param_x_1<=S_pixel_x-12'd16;
            S_param_valid_1<=I_cal_params_valid;
            if(S_pixel_y<520) begin
                S_param_y_1<=S_pixel_y-12'd464;
                S_param_level_1<=I_cal_param_levels[8:0];
            end else if(S_pixel_y<576) begin
                S_param_y_1<=S_pixel_y-12'd520;
                S_param_level_1<=I_cal_param_levels[17:9];
            end else begin
                S_param_y_1<=S_pixel_y-12'd576;
                S_param_level_1<=I_cal_param_levels[26:18];
            end
            S_param_hit_2<=S_param_hit_1;
            S_param_colour_2<=24'h131e2a;
            if(S_param_x_1<320 && S_param_y_1>=34 && S_param_y_1<38)
                S_param_colour_2<=S_param_valid_1 && S_param_x_1<S_param_level_1 ?
                                      24'h168bd7 : 24'h080f18;
            if(S_param_valid_1 && S_param_y_1>=32 && S_param_y_1<40 &&
               S_param_x_1>=S_param_knob && S_param_x_1<S_param_knob+9'd6)
                S_param_colour_2<=24'h647a94;
        end
    end
    reg        S_osd_hit;
    wire S_detect_hit;
    wire [23:0] S_detect_color;
    wire S_summary_hit;
    wire [23:0] S_summary_color;
    wire S_security_enable=SECURITY_SUPPORT ? I_security_enable : 1'b0;
    wire S_security_hit;
    wire [23:0] S_security_colour;
    generate if(SECURITY_SUPPORT) begin:g_security_overlay
        security_overlay #(.SCREEN_WIDTH(IMG_WIDTH),.SCREEN_HEIGHT(IMG_HEIGHT)) u_security(
            .clk(I_clk),.rst_n(I_rst_n),.de(I_video_de),.user(I_video_user),
            .enable(S_security_enable),.x(S_pixel_x),.y(S_pixel_y),
            .app_state(I_security_status[53:51]),.severity(I_security_status[50:48]),
            .high_count_bcd(I_security_status[47:32]),
            .suspicious_count_bcd(I_security_status[31:16]),
            .event_source_id(I_security_status[15:0]),
            .hit(S_security_hit),.colour(S_security_colour));
    end else begin:g_no_security
        assign S_security_hit=1'b0;assign S_security_colour=0;
    end endgenerate
    wire S_track_hit;
    wire [23:0] S_track_color;
    // Two stages parallel to DE_2d: no change to the DDR read/pixel latency.
    generate if(DETECT_SUPPORT) begin : g_shape_overlay
        shape_overlay #(.SCREEN_WIDTH(H_OFFSET+IMG_WIDTH),
            .SCREEN_HEIGHT(V_OFFSET+IMG_HEIGHT),.H_OFFSET(H_OFFSET),.V_OFFSET(V_OFFSET),
            .SHOW_COORDINATES(0),.BORDER_WIDTH(1),.APPLICATION_SUPPORT(SECURITY_SUPPORT))
        u_overlay(.clk(I_clk),.rst_n(I_rst_n),.de(I_video_de),
            .application_mode(S_security_enable),
            .x(S_pixel_x),.y(S_pixel_y),.boxes(I_detect_box),
            .hit(S_detect_hit),.colour(S_detect_color));
    end else begin : g_no_shape_overlay
        assign S_detect_hit=1'b0;
        assign S_detect_color=24'd0;
    end endgenerate
    generate if(DETECT_SUPPORT) begin : g_detect_summary
        detect_summary_overlay #(.PANEL_X(176)) u_summary(
            .clk(I_clk),.rst_n(I_rst_n),.de(I_video_de),.user(I_video_user),
            .enable(I_notice_detect_enable && !S_security_enable),.x(S_pixel_x),.y(S_pixel_y),
            .boxes(I_detect_box),.hit(S_summary_hit),.colour(S_summary_color));
    end else begin : g_no_detect_summary
        assign S_summary_hit=1'b0;
        assign S_summary_color=24'd0;
    end endgenerate
    generate if(TRACKING_SUPPORT) begin : g_tracking_overlay
        tracking_overlay #(.SCREEN_WIDTH(IMG_WIDTH),.SCREEN_HEIGHT(IMG_HEIGHT),
            .H_OFFSET(H_OFFSET),.V_OFFSET(V_OFFSET),.SECURITY_SUPPORT(SECURITY_SUPPORT)) u_track_overlay(
            .clk(I_clk),.rst_n(I_rst_n),.de(I_video_de),.user(I_video_user),
            .enable(I_notice_detect_enable && (!S_security_enable || I_security_status[53:51]==3'd2)),
            .hide_panel(S_security_enable),.x(S_pixel_x),.y(S_pixel_y),
            .tracked_box(I_track_box),.track_id(I_track_id),.track_state(I_track_state),
            .hit(S_track_hit),.colour(S_track_color));
    end else begin : g_no_tracking_overlay
        assign S_track_hit=1'b0;assign S_track_color=24'd0;
    end endgenerate
    reg [23:0] S_osd_color;
    wire [24:0] S_logo_pixel;
    reg [31:0] S_frame_cnt;
    reg [3:0]  S_cnt_d0;
    reg [3:0]  S_cnt_d1;
    reg [3:0]  S_cnt_d2;
    reg [3:0]  S_cnt_d3;
    reg [3:0]  S_cnt_d4;
    reg [3:0]  S_cnt_d5;
    reg [7:0] S_pf_char_0,S_pf_char_1;
    reg [3:0] S_pf_col_0,S_pf_col_1,S_pf_col_2,S_pf_row_0,S_pf_row_1;
    reg S_pf_valid_0,S_pf_valid_1,S_pf_valid_2;
    wire [15:0] S_pf_row_bits;
    reg [3:0]  S_cal_local_x;
    wire[15:0] S_cal_row_bits;
    // Use the two existing DDR/Logo latency slots for text preparation.
    // Stage 1 decodes coordinates/prefix character, stage 2 reads both glyph
    // rows synchronously; the output only selects a bit and mixes colour.
    // The existing three-clock video/DDR alignment is unchanged.
    localparam [2:0] TEXT_NONE=0, TEXT_NOTICE=1, TEXT_DETAIL=2, TEXT_SCOPE=3;
    reg [2:0] S_text_kind_0, S_text_kind_1, S_text_kind_2;
    reg [3:0] S_text_line_0, S_text_line_1;
    reg [5:0] S_text_char_col_0, S_text_char_col_1;
    reg [3:0] S_text_x_0, S_text_y_0, S_text_x_1, S_text_y_1;
    reg [1:0] S_text_colour_0, S_text_colour_1, S_text_colour_2;
    reg [2:0]   S_notice_kind;
    reg [2:0]   S_notice_value;
    reg [26:0]  S_notice_timer;
    reg [2:0]   S_notice_mode_d;
    reg         S_notice_awb_d;
    reg         S_notice_ae_d;
    reg         S_notice_detail_d;
    reg         S_notice_detect_d;

    localparam LOGO_X = 12'd0;
    localparam LOGO_Y = 12'd0;
    localparam LOGO_W = 12'd160;
    localparam LOGO_H = 12'd160;

    localparam COLOR_LOGO_BLUE   = 24'h1f315c;
    localparam COLOR_LOGO_RED    = 24'hec1c2d;
    localparam COLOR_TEXT        = 24'h00e6ff;
    localparam DYN_OSD_X         = 12'd8;
    localparam DYN_OSD_Y         = 12'd656;
    localparam DYN_OSD_W         = 12'd200;
    localparam DYN_OSD_H         = 12'd24;
    localparam CN_Y_OFFSET       = 12'd2;
    localparam COLOR_DYN_TEXT    = 24'hfff200;
    localparam COLOR_CAL_TEXT    = 24'h00e6ff;
    localparam COLOR_CAL_ROI     = 24'h00ff40;
    localparam NOTICE_X          = 12'd944;
    localparam NOTICE_Y          = 12'd16;
    localparam NOTICE_W          = 12'd320;
    localparam NOTICE_H          = 12'd40;
    localparam NOTICE_TEXT_X     = 12'd960;
    localparam NOTICE_TEXT_Y     = 12'd28;
    localparam COLOR_NOTICE_BG   = 24'h102030;
    localparam COLOR_NOTICE_TEXT = 24'hffffff;
    localparam [26:0] NOTICE_TICKS = 27'd74250000;
    localparam NOTICE_MODE       = 3'd1;
    localparam NOTICE_AWB        = 3'd2;
    localparam NOTICE_AE         = 3'd3;
    localparam NOTICE_DETAIL     = 3'd4;
    localparam NOTICE_DETECT     = 3'd5;
    localparam CAL_TEXT_X        = 12'd8;
    localparam CAL_TEXT_Y        = 12'd176;
    localparam CAL_ROI_X0        = H_OFFSET + (IMG_WIDTH-512)/2;
    localparam CAL_ROI_X1        = CAL_ROI_X0 + 511;
    localparam CAL_ROI_Y0        = V_OFFSET + (IMG_HEIGHT-256)/2;
    localparam CAL_ROI_Y1        = CAL_ROI_Y0 + 255;
    localparam CHAR_AN           = 8'h80;
    localparam CHAR_LU           = 8'h81;
    localparam CHAR_C            = 8'h43;
    localparam CHAR_N            = 8'h4e;
    localparam CHAR_T            = 8'h54;
    localparam CHAR_0            = 8'h30;
    localparam CHAR_COLON        = 8'h3a;


    // 7-seg bit mapping: {a,b,c,d,e,f,g}


    // Minimal 16x16 Chinese bitmap rows for "an" and "lu".


    // Unified prefix character library (16x16 rows): an/lu/C/N/T/:

    // Numeric inputs arrive preformatted as packed BCD, so the HDMI pixel
    // path contains no binary divider.
    // Exact floor(16*p/24), valid for the 24-pixel character cell.
    function [3:0] F_scale_24_to_16;
        input [4:0] p;
        begin
            case(p)
                5'd0: F_scale_24_to_16=4'd0;
                5'd1: F_scale_24_to_16=4'd0;
                5'd2: F_scale_24_to_16=4'd1;
                5'd3: F_scale_24_to_16=4'd2;
                5'd4: F_scale_24_to_16=4'd2;
                5'd5: F_scale_24_to_16=4'd3;
                5'd6: F_scale_24_to_16=4'd4;
                5'd7: F_scale_24_to_16=4'd4;
                5'd8: F_scale_24_to_16=4'd5;
                5'd9: F_scale_24_to_16=4'd6;
                5'd10: F_scale_24_to_16=4'd6;
                5'd11: F_scale_24_to_16=4'd7;
                5'd12: F_scale_24_to_16=4'd8;
                5'd13: F_scale_24_to_16=4'd8;
                5'd14: F_scale_24_to_16=4'd9;
                5'd15: F_scale_24_to_16=4'd10;
                5'd16: F_scale_24_to_16=4'd10;
                5'd17: F_scale_24_to_16=4'd11;
                5'd18: F_scale_24_to_16=4'd12;
                5'd19: F_scale_24_to_16=4'd12;
                5'd20: F_scale_24_to_16=4'd13;
                5'd21: F_scale_24_to_16=4'd14;
                5'd22: F_scale_24_to_16=4'd14;
                5'd23: F_scale_24_to_16=4'd15;
                default: F_scale_24_to_16=4'd0;
            endcase
        end
    endfunction

    function [7:0] F_hex_char;
        input [3:0] digit;
        begin F_hex_char = digit < 10 ? 8'h30 + digit : 8'h41 + digit - 10; end
    endfunction

    // Compact side-column statistics, normal 16x16 font.
    function [7:0] F_cal_char_at;
        input [3:0] I_line;
        input [5:0] I_col;
        begin
            F_cal_char_at=" ";
            case(I_line)
                4'd0: case(I_col)
                    0:F_cal_char_at="M";
                    1:F_cal_char_at=":";
                    2:F_cal_char_at=8'h30+I_cal_mode;
                    4:F_cal_char_at="R";
                    5:F_cal_char_at="A";
                    6:F_cal_char_at="W";
                    7:F_cal_char_at="1";
                    8:F_cal_char_at="0";
                    10:F_cal_char_at="C";
                    11:F_cal_char_at="F";
                    12:F_cal_char_at="G";
                    13:F_cal_char_at=":";
                    14:F_cal_char_at=I_cam_cfg_error ? "E" : (I_cam_cfg_ready ? "O" : "B");
                    16:F_cal_char_at="P";
                    17:F_cal_char_at="F";
                    18:F_cal_char_at=":";
                    19:F_cal_char_at=F_hex_char(I_cal_path_status);
                    default:F_cal_char_at=" ";
                endcase
                4'd1: case(I_col)
                    0:F_cal_char_at="R";
                    1:F_cal_char_at="O";
                    2:F_cal_char_at="I";
                    3:F_cal_char_at=":";
                    4:F_cal_char_at="5";
                    5:F_cal_char_at="1";
                    6:F_cal_char_at="2";
                    7:F_cal_char_at="X";
                    8:F_cal_char_at="2";
                    9:F_cal_char_at="5";
                    10:F_cal_char_at="6";
                    12:F_cal_char_at="C";
                    13:F_cal_char_at="L";
                    14:F_cal_char_at="P";
                    15:F_cal_char_at=":";
                    16:F_cal_char_at=8'h30+I_cal_clip_bcd[12+:4];
                    17:F_cal_char_at=8'h30+I_cal_clip_bcd[8+:4];
                    18:F_cal_char_at=8'h30+I_cal_clip_bcd[4+:4];
                    19:F_cal_char_at=8'h30+I_cal_clip_bcd[0+:4];
                    default:F_cal_char_at=" ";
                endcase
                4'd2: case(I_col)
                    0:F_cal_char_at="B";
                    1:F_cal_char_at=":";
                    2:F_cal_char_at=8'h30+I_cal_raw_b_bcd[12+:4];
                    3:F_cal_char_at=8'h30+I_cal_raw_b_bcd[8+:4];
                    4:F_cal_char_at=8'h30+I_cal_raw_b_bcd[4+:4];
                    5:F_cal_char_at=8'h30+I_cal_raw_b_bcd[0+:4];
                    7:F_cal_char_at="G";
                    8:F_cal_char_at="B";
                    9:F_cal_char_at=":";
                    10:F_cal_char_at=8'h30+I_cal_raw_gb_bcd[12+:4];
                    11:F_cal_char_at=8'h30+I_cal_raw_gb_bcd[8+:4];
                    12:F_cal_char_at=8'h30+I_cal_raw_gb_bcd[4+:4];
                    13:F_cal_char_at=8'h30+I_cal_raw_gb_bcd[0+:4];
                    default:F_cal_char_at=" ";
                endcase
                4'd3: case(I_col)
                    0:F_cal_char_at="G";
                    1:F_cal_char_at="R";
                    2:F_cal_char_at=":";
                    3:F_cal_char_at=8'h30+I_cal_raw_gr_bcd[12+:4];
                    4:F_cal_char_at=8'h30+I_cal_raw_gr_bcd[8+:4];
                    5:F_cal_char_at=8'h30+I_cal_raw_gr_bcd[4+:4];
                    6:F_cal_char_at=8'h30+I_cal_raw_gr_bcd[0+:4];
                    8:F_cal_char_at="R";
                    9:F_cal_char_at=":";
                    10:F_cal_char_at=8'h30+I_cal_raw_r_bcd[12+:4];
                    11:F_cal_char_at=8'h30+I_cal_raw_r_bcd[8+:4];
                    12:F_cal_char_at=8'h30+I_cal_raw_r_bcd[4+:4];
                    13:F_cal_char_at=8'h30+I_cal_raw_r_bcd[0+:4];
                    default:F_cal_char_at=" ";
                endcase
                4'd4: case(I_col)
                    0:F_cal_char_at="R";
                    1:F_cal_char_at="G";
                    2:F_cal_char_at="B";
                    4:F_cal_char_at="R";
                    5:F_cal_char_at=":";
                    6:F_cal_char_at=8'h30+I_cal_rgb_r_bcd[8+:4];
                    7:F_cal_char_at=8'h30+I_cal_rgb_r_bcd[4+:4];
                    8:F_cal_char_at=8'h30+I_cal_rgb_r_bcd[0+:4];
                    10:F_cal_char_at="G";
                    11:F_cal_char_at=":";
                    12:F_cal_char_at=8'h30+I_cal_rgb_g_bcd[8+:4];
                    13:F_cal_char_at=8'h30+I_cal_rgb_g_bcd[4+:4];
                    14:F_cal_char_at=8'h30+I_cal_rgb_g_bcd[0+:4];
                    16:F_cal_char_at="B";
                    17:F_cal_char_at=":";
                    18:F_cal_char_at=8'h30+I_cal_rgb_b_bcd[8+:4];
                    19:F_cal_char_at=8'h30+I_cal_rgb_b_bcd[4+:4];
                    20:F_cal_char_at=8'h30+I_cal_rgb_b_bcd[0+:4];
                    default:F_cal_char_at=" ";
                endcase
                4'd5: case(I_col)
                    0:F_cal_char_at="A";
                    1:F_cal_char_at="W";
                    2:F_cal_char_at="B";
                    3:F_cal_char_at=":";
                    4:F_cal_char_at=I_cal_awb_enable ? "1" : "0";
                    6:F_cal_char_at="A";
                    7:F_cal_char_at="E";
                    8:F_cal_char_at=":";
                    9:F_cal_char_at=I_cal_ae_enable ? "1" : "0";
                    11:F_cal_char_at="D";
                    12:F_cal_char_at="D";
                    13:F_cal_char_at="R";
                    14:F_cal_char_at=":";
                    15:F_cal_char_at=F_hex_char(I_link_status[7:4]);
                    default:F_cal_char_at=" ";
                endcase
                4'd6: case(I_col)
                    0:F_cal_char_at="L";
                    1:F_cal_char_at="A";
                    2:F_cal_char_at="T";
                    3:F_cal_char_at="E";
                    4:F_cal_char_at="N";
                    5:F_cal_char_at="C";
                    6:F_cal_char_at="Y";
                    7:F_cal_char_at=":";
                    9:F_cal_char_at=8'h30+I_latency_frames;
                    11:F_cal_char_at="F";
                    12:F_cal_char_at="R";
                    13:F_cal_char_at="A";
                    14:F_cal_char_at="M";
                    15:F_cal_char_at="E";
                    16:F_cal_char_at="S";
                    default:F_cal_char_at=" ";
                endcase
                4'd7: case(I_col)
                    0:F_cal_char_at="C";
                    1:F_cal_char_at="A";
                    2:F_cal_char_at="M";
                    3:F_cal_char_at=":";
                    4:F_cal_char_at=(I_frame_statistics[60] ? 8'h30+I_frame_statistics[8+:4] : "-");
                    5:F_cal_char_at=(I_frame_statistics[60] ? 8'h30+I_frame_statistics[4+:4] : "-");
                    6:F_cal_char_at=(I_frame_statistics[60] ? 8'h30+I_frame_statistics[0+:4] : "-");
                    8:F_cal_char_at="H";
                    9:F_cal_char_at="D";
                    10:F_cal_char_at="M";
                    11:F_cal_char_at="I";
                    12:F_cal_char_at=":";
                    13:F_cal_char_at=(I_frame_statistics[60] ? 8'h30+I_frame_statistics[44+:4] : "-");
                    14:F_cal_char_at=(I_frame_statistics[60] ? 8'h30+I_frame_statistics[40+:4] : "-");
                    15:F_cal_char_at=(I_frame_statistics[60] ? 8'h30+I_frame_statistics[36+:4] : "-");
                    17:F_cal_char_at="F";
                    18:F_cal_char_at="P";
                    19:F_cal_char_at="S";
                    default:F_cal_char_at=" ";
                endcase
                4'd8: case(I_col)
                    0:F_cal_char_at="D";
                    1:F_cal_char_at="E";
                    2:F_cal_char_at="T";
                    3:F_cal_char_at=":";
                    4:F_cal_char_at=(I_frame_statistics[60] ? 8'h30+I_frame_statistics[56+:4] : "-");
                    5:F_cal_char_at=(I_frame_statistics[60] ? 8'h30+I_frame_statistics[52+:4] : "-");
                    6:F_cal_char_at=(I_frame_statistics[60] ? 8'h30+I_frame_statistics[48+:4] : "-");
                    8:F_cal_char_at="N";
                    9:F_cal_char_at="E";
                    10:F_cal_char_at="W";
                    11:F_cal_char_at=":";
                    12:F_cal_char_at=(I_frame_statistics[60] ? 8'h30+I_frame_statistics[20+:4] : "-");
                    13:F_cal_char_at=(I_frame_statistics[60] ? 8'h30+I_frame_statistics[16+:4] : "-");
                    14:F_cal_char_at=(I_frame_statistics[60] ? 8'h30+I_frame_statistics[12+:4] : "-");
                    16:F_cal_char_at="R";
                    17:F_cal_char_at="E";
                    18:F_cal_char_at="P";
                    19:F_cal_char_at=":";
                    20:F_cal_char_at=(I_frame_statistics[60] ? 8'h30+I_frame_statistics[32+:4] : "-");
                    21:F_cal_char_at=(I_frame_statistics[60] ? 8'h30+I_frame_statistics[28+:4] : "-");
                    22:F_cal_char_at=(I_frame_statistics[60] ? 8'h30+I_frame_statistics[24+:4] : "-");
                    default:F_cal_char_at=" ";
                endcase
                4'd14: case(I_col)
                    0:F_cal_char_at="L";
                    1:F_cal_char_at="E";
                    2:F_cal_char_at="V";
                    3:F_cal_char_at="E";
                    4:F_cal_char_at="L";
                    6:F_cal_char_at="0";
                    7:F_cal_char_at="-";
                    8:F_cal_char_at="2";
                    9:F_cal_char_at="5";
                    10:F_cal_char_at="5";
                    default:F_cal_char_at=" ";
                endcase
                4'd12: case(I_col)
                    0:F_cal_char_at="R";
                    1:F_cal_char_at="G";
                    2:F_cal_char_at="B";
                    4:F_cal_char_at="L";
                    5:F_cal_char_at="O";
                    6:F_cal_char_at="G";
                    default:F_cal_char_at=" ";
                endcase
                4'd13: case(I_col)
                    0:F_cal_char_at="Y";
                    2:F_cal_char_at="0";
                    3:F_cal_char_at="-";
                    4:F_cal_char_at="1";
                    5:F_cal_char_at="0";
                    6:F_cal_char_at="0";
                    7:F_cal_char_at="%";
                    default:F_cal_char_at=" ";
                endcase
                // Parameter titles and right-aligned applied values below FPS/latency.
                4'd9: case(I_col)
                    0:F_cal_char_at="E";1:F_cal_char_at="X";2:F_cal_char_at="P";
                    6:F_cal_char_at="W";7:F_cal_char_at="R";8:F_cal_char_at=":";
                    9:F_cal_char_at=I_cal_params_valid ? "O" : "-";
                    10:F_cal_char_at=I_cal_params_valid ? "K" : "-";
                    16:F_cal_char_at=I_cal_params_valid ? "0"+I_cal_exposure_bcd[19:16] : "-";
                    17:F_cal_char_at=I_cal_params_valid ? "0"+I_cal_exposure_bcd[15:12] : "-";
                    18:F_cal_char_at=I_cal_params_valid ? "0"+I_cal_exposure_bcd[11:8] : "-";
                    19:F_cal_char_at=I_cal_params_valid ? "0"+I_cal_exposure_bcd[7:4] : "-";
                    20:F_cal_char_at=I_cal_params_valid ? "0"+I_cal_exposure_bcd[3:0] : "-";
                    default:F_cal_char_at=" ";
                endcase
                4'd10,4'd11: case(I_col)
                    0:F_cal_char_at=I_line==10 ? "A" : "D";
                    1:F_cal_char_at=".";2:F_cal_char_at="G";3:F_cal_char_at="A";
                    4:F_cal_char_at="I";5:F_cal_char_at="N";
                    14:F_cal_char_at=I_cal_params_valid ? "0"+(I_line==10 ? I_cal_gain_bcd[19:16] : I_cal_dgain_bcd[19:16]) : "-";
                    15:F_cal_char_at=I_cal_params_valid ? "0"+(I_line==10 ? I_cal_gain_bcd[15:12] : I_cal_dgain_bcd[15:12]) : "-";
                    16:F_cal_char_at=".";
                    17:F_cal_char_at=I_cal_params_valid ? "0"+(I_line==10 ? I_cal_gain_bcd[11:8] : I_cal_dgain_bcd[11:8]) : "-";
                    18:F_cal_char_at=I_cal_params_valid ? "0"+(I_line==10 ? I_cal_gain_bcd[7:4] : I_cal_dgain_bcd[7:4]) : "-";
                    19:F_cal_char_at=I_cal_params_valid ? "0"+(I_line==10 ? I_cal_gain_bcd[3:0] : I_cal_dgain_bcd[3:0]) : "-";
                    20:F_cal_char_at="X";
                    default:F_cal_char_at=" ";
                endcase
                default:F_cal_char_at=" ";
            endcase
            if(I_line==6 && (!I_latency_valid || I_latency_overflow)) begin
                case(I_col)
                    9:F_cal_char_at=I_latency_valid ? "O" : "W";
                    10:F_cal_char_at=I_latency_valid ? "V" : "A";
                    11:F_cal_char_at=I_latency_valid ? "E" : "I";
                    12:F_cal_char_at=I_latency_valid ? "R" : "T";
                    default:if(I_col>8) F_cal_char_at=" ";
                endcase
            end
        end
    endfunction

    function [7:0] F_notice_char_at;
        input [2:0] I_kind;
        input [2:0] I_value;
        input [4:0] I_col;
        begin
            F_notice_char_at = 8'h20;
            case(I_kind)
                NOTICE_AWB: begin
                    case(I_col)
                        0:F_notice_char_at="A"; 1:F_notice_char_at="W";
                        2:F_notice_char_at="B"; 3:F_notice_char_at=":";
                        5:F_notice_char_at="O";
                        6:F_notice_char_at=I_value[0] ? "N" : "F";
                        7:F_notice_char_at=I_value[0] ? " " : "F";
                        default:F_notice_char_at=8'h20;
                    endcase
                end
                NOTICE_AE: begin
                    case(I_col)
                        0:F_notice_char_at="A"; 1:F_notice_char_at="E";
                        2:F_notice_char_at=":"; 4:F_notice_char_at="O";
                        5:F_notice_char_at=I_value[0] ? "N" : "F";
                        6:F_notice_char_at=I_value[0] ? " " : "F";
                        default:F_notice_char_at=8'h20;
                    endcase
                end
                NOTICE_DETAIL: begin
                    case(I_col)
                        0:F_notice_char_at="D"; 1:F_notice_char_at="E";
                        2:F_notice_char_at="T"; 3:F_notice_char_at="A";
                        4:F_notice_char_at="I"; 5:F_notice_char_at="L";
                        6:F_notice_char_at=":"; 8:F_notice_char_at="O";
                        9:F_notice_char_at=I_value[0] ? "N" : "F";
                        10:F_notice_char_at=I_value[0] ? " " : "F";
                        default:F_notice_char_at=8'h20;
                    endcase
                end
                NOTICE_DETECT: begin
                    case(I_col)
                        0:F_notice_char_at="D"; 1:F_notice_char_at="E";
                        2:F_notice_char_at="T"; 3:F_notice_char_at="E";
                        4:F_notice_char_at="C"; 5:F_notice_char_at="T";
                        6:F_notice_char_at=":"; 8:F_notice_char_at="O";
                        9:F_notice_char_at=I_value[0] ? "N" : "F";
                        10:F_notice_char_at=I_value[0] ? " " : "F";
                        default:F_notice_char_at=8'h20;
                    endcase
                end
                NOTICE_MODE: begin
                    case(I_col)
                        0:F_notice_char_at="M"; 1:F_notice_char_at="O";
                        2:F_notice_char_at="D"; 3:F_notice_char_at="E";
                        4:F_notice_char_at=":";
                        6:case(I_value)
                            0:F_notice_char_at="P"; 1:F_notice_char_at="G";
                            2:F_notice_char_at="B"; 3:F_notice_char_at="S";
                            4:F_notice_char_at="E"; 5:F_notice_char_at="C";
                            6:F_notice_char_at="I"; default:F_notice_char_at="H";
                        endcase
                        7:case(I_value)
                            0:F_notice_char_at="A"; 1:F_notice_char_at="R";
                            2:F_notice_char_at="I"; 3:F_notice_char_at="O";
                            4:F_notice_char_at="D"; 5:F_notice_char_at="A";
                            6:F_notice_char_at="N"; default:F_notice_char_at="E";
                        endcase
                        8:case(I_value)
                            0:F_notice_char_at="S"; 1:F_notice_char_at="A";
                            2:F_notice_char_at="N"; 3:F_notice_char_at="B";
                            4:F_notice_char_at="G"; 5:F_notice_char_at="R";
                            6:F_notice_char_at="V"; default:F_notice_char_at="Q";
                        endcase
                        9:case(I_value)
                            0:F_notice_char_at="S"; 1:F_notice_char_at="Y";
                            2:F_notice_char_at="A"; 3:F_notice_char_at="E";
                            4:F_notice_char_at="E"; 5:F_notice_char_at="T";
                            6:F_notice_char_at="E"; default:F_notice_char_at=" ";
                        endcase
                        10:case(I_value)
                            0:F_notice_char_at=" "; 1:F_notice_char_at=" ";
                            2:F_notice_char_at="R"; 3:F_notice_char_at="L";
                            4:F_notice_char_at=" "; 5:F_notice_char_at="O";
                            6:F_notice_char_at="R"; default:F_notice_char_at=" ";
                        endcase
                        11:case(I_value)
                            2:F_notice_char_at="Y"; 5:F_notice_char_at="O";
                            6:F_notice_char_at="T";
                            default:F_notice_char_at=" ";
                        endcase
                        12:case(I_value)
                            5:F_notice_char_at="N";
                            default:F_notice_char_at=" ";
                        endcase
                        13:case(I_value)
                            default:F_notice_char_at=" ";
                        endcase
                        default:F_notice_char_at=8'h20;
                    endcase
                end
                default:F_notice_char_at=8'h20;
            endcase
        end
    endfunction

    // Read every pixel in the image window, including underneath the OSD.
    // In 720p H/V_OFFSET=0; overlay transparency must not skip DDR pixels.
    assign O_video_rd_en = S_in_image;

    // Two ERAM accesses occupy exactly the original address/pixel stages.
    anlogic_logo_rom u_anlogic_logo_rom(
        .I_clk(I_clk),.I_rst_n(I_rst_n),
        .I_x(S_pixel_x-LOGO_X),.I_y(S_pixel_y-LOGO_Y),.O_pixel(S_logo_pixel)
    );

    // Keep both glyph streams independent: a transparent prefix pixel may
    // still reveal calibration text. One true-dual-port ROM serves both.
    wire [7:0] S_cal_font_char=S_text_kind_1==TEXT_NOTICE ?
        F_notice_char_at(S_notice_kind,S_notice_value,S_text_char_col_1[4:0]) :
        F_cal_char_at(S_text_line_1,S_text_char_col_1);
    osd_mixer_font_rom u_mixer_font(
        .I_clk(I_clk),.I_char_a(S_pf_char_1),.I_row_a(S_pf_row_1),
        .I_char_b(S_cal_font_char),.I_row_b(S_text_y_1),
        .O_row_a(S_pf_row_bits),.O_row_b(S_cal_row_bits));

    // Decode the dynamic prefix directly from the undelayed pixel. Absolute
    // cell ranges avoid a wide subtract feeding another range decoder.
    always @(*) begin
        S_pf_char_0=8'h20;S_pf_col_0=0;S_pf_row_0=0;S_pf_valid_0=0;
        if(I_video_de && S_pixel_y>=DYN_OSD_Y && S_pixel_y<DYN_OSD_Y+24) begin
            if(S_pixel_x>=DYN_OSD_X && S_pixel_x<DYN_OSD_X+24 &&
               S_pixel_y>=DYN_OSD_Y+CN_Y_OFFSET) begin
                S_pf_char_0=CHAR_AN;S_pf_valid_0=1;
                S_pf_col_0=F_scale_24_to_16(S_pixel_x-DYN_OSD_X);
                S_pf_row_0=F_scale_24_to_16(S_pixel_y-DYN_OSD_Y-CN_Y_OFFSET);
            end else if(S_pixel_x>=DYN_OSD_X+24 && S_pixel_x<DYN_OSD_X+48 &&
                        S_pixel_y>=DYN_OSD_Y+CN_Y_OFFSET) begin
                S_pf_char_0=CHAR_LU;S_pf_valid_0=1;
                S_pf_col_0=F_scale_24_to_16(S_pixel_x-(DYN_OSD_X+24));
                S_pf_row_0=F_scale_24_to_16(S_pixel_y-DYN_OSD_Y-CN_Y_OFFSET);
            end else if(S_pixel_x>=DYN_OSD_X+52 && S_pixel_x<DYN_OSD_X+100 &&
                        S_pixel_y>=DYN_OSD_Y+4 && S_pixel_y<DYN_OSD_Y+20) begin
                S_pf_valid_0=1;S_pf_row_0=S_pixel_y-(DYN_OSD_Y+4);
                if(S_pixel_x<DYN_OSD_X+68) begin
                    S_pf_char_0=CHAR_C;S_pf_col_0=S_pixel_x-(DYN_OSD_X+52);
                end else if(S_pixel_x<DYN_OSD_X+84) begin
                    S_pf_char_0=CHAR_N;S_pf_col_0=S_pixel_x-(DYN_OSD_X+68);
                end else begin
                    S_pf_char_0=CHAR_T;S_pf_col_0=S_pixel_x-(DYN_OSD_X+84);
                end
            end else if(S_pixel_x>=DYN_OSD_X+100 && S_pixel_x<DYN_OSD_X+108 &&
                        S_pixel_y>=DYN_OSD_Y+4 && S_pixel_y<DYN_OSD_Y+20) begin
                S_pf_char_0=CHAR_COLON;S_pf_valid_0=1;
                S_pf_col_0=(S_pixel_x-(DYN_OSD_X+100))<<1;
                S_pf_row_0=S_pixel_y-(DYN_OSD_Y+4);
            end else if(S_pixel_x>=DYN_OSD_X+110 && S_pixel_x<DYN_OSD_X+200) begin
                S_pf_valid_0=1;S_pf_row_0=F_scale_24_to_16(S_pixel_y-DYN_OSD_Y);
                if(S_pixel_x<DYN_OSD_X+125) begin
                    S_pf_char_0=CHAR_0+S_cnt_d5;S_pf_col_0=S_pixel_x-(DYN_OSD_X+110);
                end else if(S_pixel_x<DYN_OSD_X+140) begin
                    S_pf_char_0=CHAR_0+S_cnt_d4;S_pf_col_0=S_pixel_x-(DYN_OSD_X+125);
                end else if(S_pixel_x<DYN_OSD_X+155) begin
                    S_pf_char_0=CHAR_0+S_cnt_d3;S_pf_col_0=S_pixel_x-(DYN_OSD_X+140);
                end else if(S_pixel_x<DYN_OSD_X+170) begin
                    S_pf_char_0=CHAR_0+S_cnt_d2;S_pf_col_0=S_pixel_x-(DYN_OSD_X+155);
                end else if(S_pixel_x<DYN_OSD_X+185) begin
                    S_pf_char_0=CHAR_0+S_cnt_d1;S_pf_col_0=S_pixel_x-(DYN_OSD_X+170);
                end else begin
                    S_pf_char_0=CHAR_0+S_cnt_d0;S_pf_col_0=S_pixel_x-(DYN_OSD_X+185);
                end
            end
        end
    end

    // Coordinate decoding is independent of the late OSD hit priority chain.
    // Decode from the undelayed pixel so metadata reaches the same _2d pixel
    // as the synchronous Logo RAM, scopes and detector overlays.
    always @(*) begin
        S_text_kind_0=TEXT_NONE;
        S_text_line_0=0; S_text_char_col_0=0;
        S_text_x_0=0; S_text_y_0=0; S_text_colour_0=0;
        if(I_video_de) begin
            if(S_pixel_x>=NOTICE_TEXT_X && S_pixel_x<NOTICE_TEXT_X+12'd288 &&
               S_pixel_y>=NOTICE_TEXT_Y && S_pixel_y<NOTICE_TEXT_Y+12'd16) begin
                S_text_kind_0=TEXT_NOTICE;
                S_text_char_col_0=(S_pixel_x-NOTICE_TEXT_X)>>4;
                S_text_x_0=S_pixel_x-NOTICE_TEXT_X;
                S_text_y_0=S_pixel_y-NOTICE_TEXT_Y;
            end else if(S_pixel_x>=CAL_TEXT_X && S_pixel_x<CAL_TEXT_X+12'd384 &&
                        S_pixel_y>=CAL_TEXT_Y && S_pixel_y<CAL_TEXT_Y+12'd288 &&
                        ((S_pixel_y-CAL_TEXT_Y)&12'd31)<16) begin
                S_text_kind_0=TEXT_DETAIL;
                S_text_line_0=(S_pixel_y-CAL_TEXT_Y)>>5;
                S_text_char_col_0=(S_pixel_x-CAL_TEXT_X)>>4;
                S_text_x_0=S_pixel_x-CAL_TEXT_X;
                S_text_y_0=S_pixel_y-CAL_TEXT_Y;
            end else if(S_pixel_x>=CAL_TEXT_X && S_pixel_x<CAL_TEXT_X+12'd336 &&
                         ((S_pixel_y>=472 && S_pixel_y<488) ||
                          (S_pixel_y>=528 && S_pixel_y<544) ||
                          (S_pixel_y>=584 && S_pixel_y<600))) begin
                S_text_kind_0=TEXT_DETAIL;
                S_text_line_0=S_pixel_y<528 ? 9 : S_pixel_y<584 ? 10 : 11;
                S_text_char_col_0=(S_pixel_x-CAL_TEXT_X)>>4;
                S_text_x_0=S_pixel_x-CAL_TEXT_X;
                S_text_y_0=S_pixel_y<528 ? S_pixel_y-12'd472 :
                           S_pixel_y<584 ? S_pixel_y-12'd528 : S_pixel_y-12'd584;
            end else if(S_pixel_x>=1008 && S_pixel_x<1264) begin
                S_text_char_col_0=(S_pixel_x-12'd1008)>>4;
                S_text_x_0=S_pixel_x-12'd1008;
                if(S_pixel_y>=176 && S_pixel_y<432 &&
                   ((S_pixel_y-12'd176)&12'd127)<16) begin
                    S_text_kind_0=TEXT_SCOPE;
                    S_text_line_0=12+((S_pixel_y-12'd176)>>7);
                    S_text_y_0=S_pixel_y-12'd176;
                    if(S_pixel_y<192) begin
                        case(S_text_char_col_0)
                            0:S_text_colour_0=1;
                            1:S_text_colour_0=2;
                            2:S_text_colour_0=3;
                            default:S_text_colour_0=0;
                        endcase
                    end
                end else if(S_pixel_y>=432 && S_pixel_y<448) begin
                    S_text_kind_0=TEXT_SCOPE;
                    S_text_line_0=14;
                    S_text_y_0=S_pixel_y-12'd432;
                end
            end
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_text_kind_1<=TEXT_NONE; S_text_kind_2<=TEXT_NONE;
            S_text_line_1<=0; S_text_char_col_1<=0;
            S_text_x_1<=0; S_text_y_1<=0;
            S_text_colour_1<=0; S_text_colour_2<=0;
            S_cal_local_x<=0;
            S_pf_char_1<=8'h20;S_pf_row_1<=0;S_pf_col_1<=0;S_pf_col_2<=0;
            S_pf_valid_1<=0;S_pf_valid_2<=0;
        end else begin
            S_text_kind_1<=S_text_kind_0;
            S_text_line_1<=S_text_line_0; S_text_char_col_1<=S_text_char_col_0;
            S_text_x_1<=S_text_x_0; S_text_y_1<=S_text_y_0;
            S_text_colour_1<=S_text_colour_0;
            S_text_kind_2<=S_text_kind_1;
            S_text_colour_2<=S_text_colour_1;
            S_cal_local_x<=S_text_x_1;
            S_pf_char_1<=S_pf_char_0;S_pf_row_1<=S_pf_row_0;S_pf_col_1<=S_pf_col_0;
            S_pf_col_2<=S_pf_col_1;
            S_pf_valid_1<=S_pf_valid_0;S_pf_valid_2<=S_pf_valid_1;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
        end
        else if(I_video_user) begin
            S_x <= I_video_de ? 12'd1 : 12'd0;
            S_y <= 12'd0;
        end
        else if(I_video_de) begin
            if(I_video_last) begin
                S_x <= 12'd0;
                S_y <= S_y + 12'd1;
            end
            else begin
                S_x <= S_x + 12'd1;
            end
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_frame_cnt <= 32'd0;
            S_cnt_d0 <= 4'd0;
            S_cnt_d1 <= 4'd0;
            S_cnt_d2 <= 4'd0;
            S_cnt_d3 <= 4'd0;
            S_cnt_d4 <= 4'd0;
            S_cnt_d5 <= 4'd0;
        end
        else if(I_video_user) begin
            S_frame_cnt <= S_frame_cnt + 32'd1;
            if(S_cnt_d0 == 4'd9) begin
                S_cnt_d0 <= 4'd0;
                if(S_cnt_d1 == 4'd9) begin
                    S_cnt_d1 <= 4'd0;
                    if(S_cnt_d2 == 4'd9) begin
                        S_cnt_d2 <= 4'd0;
                        if(S_cnt_d3 == 4'd9) begin
                            S_cnt_d3 <= 4'd0;
                            if(S_cnt_d4 == 4'd9) begin
                                S_cnt_d4 <= 4'd0;
                                if(S_cnt_d5 == 4'd9)
                                    S_cnt_d5 <= 4'd0;
                                else
                                    S_cnt_d5 <= S_cnt_d5 + 4'd1;
                            end
                            else
                                S_cnt_d4 <= S_cnt_d4 + 4'd1;
                        end
                        else
                            S_cnt_d3 <= S_cnt_d3 + 4'd1;
                    end
                    else
                        S_cnt_d2 <= S_cnt_d2 + 4'd1;
                end
                else
                    S_cnt_d1 <= S_cnt_d1 + 4'd1;
            end
            else
                S_cnt_d0 <= S_cnt_d0 + 4'd1;
        end
    end

    // One-second, last-action-wins notification state.  The reset values match
    // the control defaults so power-up itself does not create a false notice.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_notice_kind     <= 3'd0;
            S_notice_value    <= 3'd0;
            S_notice_timer    <= 27'd0;
            S_notice_mode_d   <= 3'd0;
            S_notice_awb_d    <= 1'b1;
            S_notice_ae_d     <= 1'b1;
            S_notice_detail_d <= 1'b0;
            S_notice_detect_d <= 1'b0;
        end
        else begin
            S_notice_mode_d   <= I_cal_mode;
            S_notice_awb_d    <= I_cal_awb_enable;
            S_notice_ae_d     <= I_cal_ae_enable;
            S_notice_detail_d <= I_notice_detail_enable;
            S_notice_detect_d <= I_notice_detect_enable;

            if(I_notice_detect_enable != S_notice_detect_d) begin
                S_notice_kind  <= NOTICE_DETECT;
                S_notice_value <= {2'd0,I_notice_detect_enable};
                S_notice_timer <= NOTICE_TICKS - 1'b1;
            end
            else if(I_notice_detail_enable != S_notice_detail_d) begin
                S_notice_kind  <= NOTICE_DETAIL;
                S_notice_value <= {2'd0,I_notice_detail_enable};
                S_notice_timer <= NOTICE_TICKS - 1'b1;
            end
            else if(I_cal_ae_enable != S_notice_ae_d) begin
                S_notice_kind  <= NOTICE_AE;
                S_notice_value <= {2'd0,I_cal_ae_enable};
                S_notice_timer <= NOTICE_TICKS - 1'b1;
            end
            else if(I_cal_awb_enable != S_notice_awb_d) begin
                S_notice_kind  <= NOTICE_AWB;
                S_notice_value <= {2'd0,I_cal_awb_enable};
                S_notice_timer <= NOTICE_TICKS - 1'b1;
            end
            else if(I_cal_mode != S_notice_mode_d) begin
                S_notice_kind  <= NOTICE_MODE;
                S_notice_value <= I_cal_mode;
                S_notice_timer <= NOTICE_TICKS - 1'b1;
            end
            else if(S_notice_timer != 0)
                S_notice_timer <= S_notice_timer - 1'b1;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x_1d <= 12'd0;
            S_x_2d <= 12'd0;
            S_y_1d <= 12'd0;
            S_y_2d <= 12'd0;
            S_in_image_1d<=0;S_in_image_2d<=0;
            S_logo_region_1<=0;S_logo_region_2<=0;
            S_notice_region_1<=0;S_notice_region_2<=0;
            S_roi_border_1<=0;S_roi_border_2<=0;
        end
        else begin
            S_in_image_1d<=S_in_image;S_in_image_2d<=S_in_image_1d;
            S_logo_region_1<=I_video_de && S_pixel_x>=LOGO_X && S_pixel_x<LOGO_X+LOGO_W &&
                S_pixel_y>=LOGO_Y && S_pixel_y<LOGO_Y+LOGO_H;
            S_logo_region_2<=S_logo_region_1;
            S_notice_region_1<=I_video_de && S_pixel_x>=NOTICE_X && S_pixel_x<NOTICE_X+NOTICE_W &&
                S_pixel_y>=NOTICE_Y && S_pixel_y<NOTICE_Y+NOTICE_H;
            S_notice_region_2<=S_notice_region_1;
            S_roi_border_1<=I_video_de && S_pixel_x>=CAL_ROI_X0 && S_pixel_x<=CAL_ROI_X1 &&
                S_pixel_y>=CAL_ROI_Y0 && S_pixel_y<=CAL_ROI_Y1 &&
                (S_pixel_x==CAL_ROI_X0 || S_pixel_x==CAL_ROI_X0+1 ||
                 S_pixel_x==CAL_ROI_X1 || S_pixel_x==CAL_ROI_X1-1 ||
                 S_pixel_y==CAL_ROI_Y0 || S_pixel_y==CAL_ROI_Y0+1 ||
                 S_pixel_y==CAL_ROI_Y1 || S_pixel_y==CAL_ROI_Y1-1);
            S_roi_border_2<=S_roi_border_1;
            S_x_1d <= S_pixel_x;
            S_x_2d <= S_x_1d;
            S_y_1d <= S_pixel_y;
            S_y_2d <= S_y_1d;
        end
    end

    always @(*) begin
        S_osd_hit   = 1'b0;
        S_osd_color = 24'd0;

        // Logo remains visible in every mode; detection panels sit beside it.
        if(S_video_de_2d) begin
            if(S_logo_region_2) begin
                if(S_logo_pixel[24]) begin
                    S_osd_hit   = 1'b1;
                    S_osd_color = S_logo_pixel[23:0];
                end
            end

            // Right-top operation banner.  It has an opaque dark backing so
            // the short status message remains readable on any camera scene.
            if(!S_security_enable && (S_notice_timer != 0) && S_notice_region_2) begin
                S_osd_hit   = 1'b1;
                S_osd_color = COLOR_NOTICE_BG;
                if(S_text_kind_2==TEXT_NOTICE && S_cal_row_bits[4'd15-S_cal_local_x])
                    S_osd_color = COLOR_NOTICE_TEXT;
            end

            if(I_cal_enable && !S_osd_hit && S_pf_valid_2 &&
               S_pf_row_bits[4'd15-S_pf_col_2]) begin
                S_osd_hit=1;S_osd_color=COLOR_DYN_TEXT;
            end

            // Both synchronous glyph reads completed in the preceding slot.
            // The output stage contains only a bit select and colour priority.
            if(I_cal_enable && !S_osd_hit && S_text_kind_2==TEXT_DETAIL &&
               S_cal_row_bits[4'd15-S_cal_local_x]) begin
                S_osd_hit=1; S_osd_color=COLOR_CAL_TEXT;
            end
            if(I_cal_enable && !S_osd_hit && S_text_kind_2==TEXT_SCOPE &&
               S_cal_row_bits[4'd15-S_cal_local_x]) begin
                S_osd_hit=1;
                case(S_text_colour_2)
                    1:S_osd_color=24'he85c62;
                    2:S_osd_color=24'h54ca89;
                    3:S_osd_color=24'h579ced;
                    default:S_osd_color=24'he8edf4;
                endcase
            end

            // The ROI coordinate comparison is also completed before this
            // final colour stage, retaining the same two-pixel border.
            if(I_cal_enable && !S_osd_hit && S_roi_border_2) begin
                S_osd_hit=1;S_osd_color=COLOR_CAL_ROI;
            end
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_video_vsync_1d <= 1'b0;
            S_video_vsync_2d <= 1'b0;
            S_video_hsync_1d <= 1'b0;
            S_video_hsync_2d <= 1'b0;
            S_video_de_1d    <= 1'b0;
            S_video_de_2d    <= 1'b0;
            O_hdmi_vsync     <= 1'b0;
            O_hdmi_hsync     <= 1'b0;
            O_hdmi_de        <= 1'b0;
            O_hdmi_data      <= 24'd0;
        end
        else begin
            S_video_vsync_1d <= I_video_vsync;
            S_video_vsync_2d <= S_video_vsync_1d;
            O_hdmi_vsync     <= S_video_vsync_2d;

            S_video_hsync_1d <= I_video_hsync;
            S_video_hsync_2d <= S_video_hsync_1d;
            O_hdmi_hsync     <= S_video_hsync_2d;

            S_video_de_1d    <= I_video_de;
            S_video_de_2d    <= S_video_de_1d;
            O_hdmi_de        <= S_video_de_2d;

            if(DEBUG_MODE == 1) begin
                if(S_video_de_2d) begin
                    if(S_x < 12'd128)
                        O_hdmi_data <= 24'hff0000;
                    else if(S_x < 12'd256)
                        O_hdmi_data <= 24'h00ff00;
                    else if(S_x < 12'd384)
                        O_hdmi_data <= 24'h0000ff;
                    else if(S_x < 12'd512)
                        O_hdmi_data <= 24'hffff00;
                    else if(S_x < 12'd640)
                        O_hdmi_data <= 24'h00ffff;
                    else if(S_x < 12'd768)
                        O_hdmi_data <= 24'hff00ff;
                    else if(S_x < 12'd896)
                        O_hdmi_data <= 24'hffffff;
                    else
                        O_hdmi_data <= 24'h202020;
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 2) begin
                if(S_video_de_2d) begin
                    case (I_debug_status)
                        4'b0000: O_hdmi_data <= 24'h000000;
                        4'b0001: O_hdmi_data <= 24'hff0000;
                        4'b0011: O_hdmi_data <= 24'hffff00;
                        4'b0111: O_hdmi_data <= 24'h00ff00;
                        4'b1111: O_hdmi_data <= I_video_rd_data;
                        default: O_hdmi_data <= 24'h0000ff;
                    endcase
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 3) begin
                if(S_video_de_2d) begin
                    if(S_x < 12'd256)
                        O_hdmi_data <= I_debug_status[0] ? 24'hff8000 : 24'h201000;
                    else if(S_x < 12'd512)
                        O_hdmi_data <= I_debug_status[1] ? 24'hffff00 : 24'h202000;
                    else if(S_x < 12'd768)
                        O_hdmi_data <= I_debug_status[2] ? 24'hff0000 : 24'h200000;
                    else
                        O_hdmi_data <= I_debug_status[3] ? 24'h00ffff : 24'h002020;
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else begin
                if(S_security_hit)
                    O_hdmi_data <= S_security_colour;
                else if(S_summary_hit)
                    O_hdmi_data <= S_summary_color;
                else if(S_track_hit)
                    O_hdmi_data <= S_track_color;
                // Opaque target panels stay legible when SW2 diagnostics are
                // also enabled. Logo/notice occupy separate screen regions.
                else if(S_osd_hit)
                    O_hdmi_data <= S_osd_color;
                else if(S_param_hit_2)
                    O_hdmi_data <= S_param_colour_2;
                else if(S_scopes_hit)
                    O_hdmi_data <= S_scopes_colour;
                else if(!S_in_image_2d)
                    O_hdmi_data <= 24'd0;
                else if(S_detect_hit)
                    O_hdmi_data <= S_detect_color;
                else
                    // video_out already has the two cycles matched by DE_2d.
                    // Delaying its result again shifts pixels and drops line tails.
                    O_hdmi_data <= I_video_rd_data;
            end
        end
    end

endmodule
