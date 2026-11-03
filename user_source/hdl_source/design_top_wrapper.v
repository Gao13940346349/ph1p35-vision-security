

module design_top_wrapper (
    input wire        I_sys_clk,
      
    output wire       O_cam_scl,
    inout  wire       IO_cam_sda,
    output wire       O_cam_24m,
    output wire       O_cam_rst,
      
    input wire [9:0] I_keys_n, // Logical K1..K10, active low; physical mapping in pin.adc
    output wire       O_buzzer, // Board passive buzzer, C12, idle low
    output wire       O_servo_pan_pwm, // J2-1/C18：水平舵机，非检测模式回正
    output wire       O_servo_tilt_pwm,// J2-2/D15：独立俯仰舵机

    output wire       O_tmds_ch0_p,
    output wire       O_tmds_ch1_p,
    output wire       O_tmds_ch2_p,
    output wire       O_tmds_clk_p,

    output wire       O_eth_phy_reset_n,

    inout wire        IO_rx_clk_pad_n, 
    inout wire        IO_rx_clk_pad_p, 
    inout wire[3:0]   IO_rx_data_pad_n,
    inout wire[3:0]   IO_rx_data_pad_p,

    output wire[12:0] ddr_addr,
    output wire[ 1:0] ddr_ba,
    output wire[ 0:0] ddr_cke,
    output wire[ 0:0] ddr_odt,
    output wire[ 0:0] ddr_cs_n,
    output wire       ddr_ras_n,
    output wire       ddr_cas_n,
    output wire       ddr_we_n,
    output wire[ 0:0] ddr_ck_p,
    output wire[ 0:0] ddr_ck_n,
    inout wire[1:0]   ddr_dm,
    inout wire[15:0]  ddr_dq,
    inout wire[1:0]   ddr_dqs_p,
    inout wire[1:0]   ddr_dqs_n   
);


    localparam integer CAM_WIDTH = 1280;
    localparam integer CAM_HEIGHT = 720;
    wire            S_hdmi_pll_lock;
    wire        	S_100m_clk;
    wire        	S_24m_clk;
    wire        	S_aux_50m_clk;
	wire        	S_10m_clk;
		
	wire        	S_pll_lock;
    wire        	S_rst_n;

    // 由 control_state 驱动（本地按键控制状态）
    wire [2:0]      S_mode;
    reg  [2:0]      S_mode_s0;       // 以下为到 MIPI 字节时钟域的同步链
    reg  [2:0]      S_mode_s1;
    reg  [2:0]      S_mode_s2;
    reg  [2:0]      S_mode_isp;
    reg             S_awb_en_s0;
    reg             S_awb_en_s1;
    reg             S_ae_en_s0;
    reg             S_ae_en_s1;
    reg             S_detect_en_s0;
    reg             S_detect_en_s1;
    wire            S_awb_enable;
    wire            S_ae_enable;
    wire            S_cal_enable;
    wire            S_detect_enable;
    wire            S_application_enable;
    wire [2:0]      S_security_tone_level;

    // Local keypad events feed one local_control_state instance.
    wire        S_btn_awb_evt, S_btn_ae_evt, S_btn_cal_evt, S_btn_detect_evt;
    wire        S_btn_mode_evt, S_btn_application_evt;


    // Manual image controls: held in the 100 MHz control layer, captured into
    // the 24 MHz camera-config domain on param_toggle.
    wire [11:0] S_manual_exposure;
    wire [6:0]  S_manual_again, S_manual_dgain;
    wire        S_manual_exposure_en, S_manual_again_en, S_manual_dgain_en;
    wire        S_param_toggle;
    reg  [2:0]  S_param_toggle_sync_24m;
    reg         S_param_toggle_seen_24m;
    reg  [11:0] S_ae_man_24m;
    reg  [6:0]  S_ag_man_24m, S_dg_man_24m;
    reg  [2:0]  S_manual_en_24m;


    wire [7:0]      S_ae_luma_avg;
    wire            S_ae_luma_toggle;
    reg  [7:0]      S_ae_luma_s0;
    reg  [7:0]      S_ae_luma_s1;
    reg             S_ae_toggle_s0;
    reg             S_ae_toggle_s1;
    reg             S_ae_toggle_s2;
    wire            S_ae_luma_valid = S_ae_toggle_s1 ^ S_ae_toggle_s2;
	
	wire [15:0] 	S_ae;
    wire [15:0] 	S_ag;
    wire [15:0] 	S_dg;
    wire 			S_cam_cfg_done;
    wire 			S_ae_cfg_done;
    wire            S_cam_cfg_error;
    reg [2:0]       S_cam_status_meta;
    reg [2:0]       S_cam_status_hdmi;
    reg [2:0]       S_mode_hdmi_meta, S_mode_hdmi_sync, S_mode_hdmi_prev, S_mode_hdmi;
    reg             S_isp_nonzero_seen;
    reg [1:0]       S_isp_nonzero_sync;
    wire 			S_ae_req; 
	
	wire        	S_csi_rx_clk;  
    wire        	S_hs_rx_valid;  
    wire[15:0]  	S_hs_rx_data;   
	wire[1:0]   	S_lane_error;
	
	wire        	S_csi_frame_start;       
	wire        	S_csi_frame_end;         
	wire        	S_csi_valid;             
	wire[31:0]  	S_csi_data;  
		
	wire        	S_raw10_frame_start; 
	wire        	S_raw10_frame_end;   
	wire        	S_raw10_valid;       
	wire [39:0] 	S_raw10_data;
		
	wire 			S_axis_tlast;  
	wire 			S_axis_tuser;  
	wire [39:0]		S_axis_tdata;  
	wire 			S_axis_tvalid; 
	
	wire [39:0]		S_raw_tdata ;
	wire			S_raw_tlast ;
	wire 			S_raw_tvalid;
	wire 			S_raw_tuser ;
	
	wire			S_ISP_O_tready;
	wire [127:0]	S_ISP_O_tdata ;
	wire 			S_ISP_O_tlast ;
	wire 			S_ISP_O_tuser ;
	wire 			S_ISP_O_tvalid; 

    wire [9:0]      S_cal_raw_b_avg;
wire [9:0]      S_cal_raw_gb_avg;
wire [9:0]      S_cal_raw_gr_avg;
wire [9:0]      S_cal_raw_r_avg;
wire [9:0]      S_cal_raw_clip_permille;
    wire            S_cal_raw_toggle;
    wire [7:0]      S_cal_rgb_r_avg;
    wire [7:0]      S_cal_rgb_g_avg;
    wire [7:0]      S_cal_rgb_b_avg;
    wire [9:0]      S_cal_clip_permille;
    wire            S_cal_rgb_toggle;
	

    wire        S_ddr_clk;
    wire        S_hdmi_pixel_clk;
    wire        S_hdmi_serial_clk;
    wire        S_hdmi_rst_n;
    wire        S_video_out_rst_n;


    wire        S_vi_128b_frame_start;
    wire        S_vi_128b_valid;      
    wire[127:0] S_vi_128b_data;    
    
    wire        S_video_out_rd_busy;
    wire        S_video_in_wr_busy; 
    wire[1:0]   S_video_out_rp;     

    wire        S_ddr_user_wr_en;       
    wire        S_ddr_user_rd_en;       
    wire[24:0]  S_ddr_user_addr;        
    wire[127:0] S_ddr_user_wr_data;     
    wire        S_ddr_user_ready;       
    wire        S_ddr_user_rd_valid;    
    wire[127:0] S_ddr_user_rd_data;     

    wire        S_vi_ddr_wr_en;   
    wire[24:0]  S_vi_ddr_wr_addr; 
    wire[127:0] S_vi_ddr_wr_data;
    wire        S_vi_ddr_wr_ready;

    wire        S_vo_ddr_rd_en;
    wire[24:0]  S_vo_ddr_rd_addr;
    wire        S_vo_ddr_rd_ready;
    reg [7:0]   S_link_status_meta;
    reg [7:0]   S_link_status_sync;
    wire        S_vo_dbg_fifo_rst;
    wire        S_vo_dbg_ddr_rd_valid;
    wire        S_vo_dbg_fifo_rd_en;
    wire        S_vo_dbg_fifo_empty;

    wire        S_init_calib_complete; 
    wire[24:0]  S_mc_app_addr;          
    wire[2:0]   S_mc_app_cmd;           
    wire        S_mc_app_en;            
    wire[127:0] S_mc_app_wdf_data;      
    wire        S_mc_app_wdf_end;       
    wire[15:0]  S_mc_app_wdf_mask;      
    wire        S_mc_app_wdf_wren;      
    wire[127:0] S_mc_app_rd_data;       
    wire        S_mc_app_rd_data_end;   
    wire        S_mc_app_rd_data_valid; 
    wire        S_mc_app_rdy;           
    wire        S_mc_app_wdf_rdy;       
    wire        S_mc_dbg_fifo_afull;
    wire        S_mc_dbg_app_rdy;
    wire        S_mc_dbg_wdf_rdy;
    wire        S_mc_dbg_cmd_pop;

    wire        S_hdmi_vsync;
    wire        S_hdmi_hsync;
    wire        S_hdmi_de;
    wire        S_hdmi_user;
    wire        S_hdmi_last;
    wire        S_video_out_vsync;
    wire        S_hdmi_window_rd_en;
    wire        S_hdmi_out_vsync;
    wire        S_hdmi_out_hsync;
    wire        S_hdmi_out_de;
    wire[23:0]  S_hdmi_out_data;

    wire[23:0]  S_video_rd_data;

    // 手动标定统计跨到 HDMI 像素时钟域。源端数据在相邻两次 toggle
    // 之间保持一整帧稳定，两级采样后再锁存可避免多位总线撕裂。
    reg  [39:0] S_cal_raw_bus_s0;
    reg  [39:0] S_cal_raw_bus_s1;
    reg         S_cal_raw_tog_s0;
    reg         S_cal_raw_tog_s1;
    reg         S_cal_raw_tog_s2;
    reg  [39:0] S_cal_raw_bus_hdmi;
    reg  [33:0] S_cal_rgb_bus_s0;
    reg  [33:0] S_cal_rgb_bus_s1;
    reg         S_cal_rgb_tog_s0;
    reg         S_cal_rgb_tog_s1;
    reg         S_cal_rgb_tog_s2;
    reg  [33:0] S_cal_rgb_bus_hdmi;
    reg  [15:0] S_cal_ae_s0;
    reg  [15:0] S_cal_ae_s1;
    reg  [15:0] S_cal_ag_s0;
    reg  [15:0] S_cal_ag_s1;
    reg  [9:0]  S_cal_raw_clip_s0;
    reg  [9:0]  S_cal_raw_clip_s1;
    reg         S_cal_en_s0;
    reg         S_cal_en_s1;
    reg         S_cal_awb_s0;
    reg         S_cal_awb_s1;
    reg         S_cal_ae_en_s0;
    reg         S_cal_ae_en_s1;

    wire [19:0] S_cal_raw_b_bcd;
    wire [19:0] S_cal_raw_gb_bcd;
    wire [19:0] S_cal_raw_gr_bcd;
    wire [19:0] S_cal_raw_r_bcd;
    wire [19:0] S_cal_rgb_r_bcd;
    wire [19:0] S_cal_rgb_g_bcd;
    wire [19:0] S_cal_rgb_b_bcd;
    wire [19:0] S_cal_clip_bcd;
    wire [19:0] S_cal_exposure_bcd;
    wire [19:0] S_cal_gain_bcd, S_cal_dgain_bcd;
    wire S_camera_cfg_rst_n;
    wire [63:0] S_camera_applied, S_camera_applied_hdmi;
    wire [15:0] S_again_milli, S_dgain_milli;
    detect_mailbox #(.WIDTH(64)) u_camera_params_snapshot(
        .rst_n(S_camera_cfg_rst_n),.src_clk(S_24m_clk),.send(1'b1),
        .data_in(S_camera_applied),.ready(),.dst_clk(S_hdmi_pixel_clk),
        .receive(),.data_out(S_camera_applied_hdmi));
    camera_gain_values u_camera_gain_values(.clk(S_hdmi_pixel_clk),
        .again(S_camera_applied_hdmi[31:16]),.dgain(S_camera_applied_hdmi[15:0]),
        .again_milli(S_again_milli),.dgain_milli(S_dgain_milli));
    // Match the two-cycle gain decode so exposure and gain share one snapshot.
    reg [15:0] S_applied_exp_d0, S_applied_exp_d1;
    reg [31:0] S_applied_gains_d0, S_applied_gains_d1;
    always @(posedge S_hdmi_pixel_clk) begin
        S_applied_exp_d0 <= S_camera_applied_hdmi[47:32];
        S_applied_exp_d1 <= S_applied_exp_d0;
        S_applied_gains_d0 <= S_camera_applied_hdmi[31:0];
        S_applied_gains_d1 <= S_applied_gains_d0;
    end
    reg         S_dbg_raw_seen;
    reg         S_dbg_isp_seen;
    reg         S_dbg_ddr_wr_seen;
    reg         S_dbg_ddr_rd_seen;
    reg         S_dbg_cam_cfg_seen;
    reg         S_dbg_hs_seen;
    reg         S_dbg_csi_seen;
    reg         S_dbg_raw10_seen;
    reg         S_dbg_isp_fs_seen;
    reg         S_dbg_isp_valid_seen;
    reg         S_dbg_ddr_req_seen;
    reg         S_dbg_ddr_ready_seen;
    reg [1:0]   S_dbg_raw_seen_sync;
    reg [1:0]   S_dbg_isp_seen_sync;
    reg [1:0]   S_dbg_ddr_wr_seen_sync;
    reg [1:0]   S_dbg_ddr_rd_seen_sync;
    reg [1:0]   S_dbg_cam_cfg_seen_sync;
    reg [1:0]   S_dbg_hs_seen_sync;
    reg [1:0]   S_dbg_csi_seen_sync;
    reg [1:0]   S_dbg_raw10_seen_sync;
    reg [1:0]   S_dbg_isp_fs_seen_sync;
    reg [1:0]   S_dbg_isp_valid_seen_sync;
    reg [1:0]   S_dbg_ddr_req_seen_sync;
    reg [1:0]   S_dbg_ddr_ready_seen_sync;
    reg         S_dbg_video_nonzero_seen;
    reg         S_dbg_rd_req_seen;
    reg         S_dbg_rd_valid_seen;
    reg         S_dbg_rd_nonzero_seen;
    reg [1:0]   S_dbg_rd_req_seen_sync;
    reg [1:0]   S_dbg_rd_valid_seen_sync;
    reg [1:0]   S_dbg_rd_nonzero_seen_sync;
    reg         S_dbg_mc_init_seen;
    reg         S_dbg_mc_app_rdy_seen;
    reg         S_dbg_mc_wdf_rdy_seen;
    reg         S_dbg_mc_cmd_pop_seen;
    reg [1:0]   S_dbg_mc_init_seen_sync;
    reg [1:0]   S_dbg_mc_app_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_wdf_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_cmd_pop_seen_sync;
    wire[3:0]   S_hdmi_debug_status;
    wire        S_lane_error_any;


    assign S_video_out_vsync = ~S_hdmi_vsync;
	
	
    local_reset_release u_camera_cfg_reset(.clk(S_24m_clk),
        .async_reset_n(S_rst_n),.reset_n(S_camera_cfg_rst_n));
    // Power/PLL reset only. No external key drives the global reset network.
    assign S_rst_n = S_pll_lock;
    assign S_hdmi_rst_n = S_rst_n & S_hdmi_pll_lock;

    wire [9:0] S_key_press;
    keypad10 u_keypad(.clk(S_100m_clk),.rst_n(S_rst_n),
        .keys_n(I_keys_n),.press(S_key_press));
    // All ten keys share accepted, debounced events and 0.2 s feedback.
    keypad_tone_feedback #(.CLK_HZ(100000000),.TONE_MS(200),.SECURITY_SUPPORT(1)) u_key_tones(
        .clk(S_100m_clk),.pll_locked(S_pll_lock),
        .press_pulse(S_key_press),
        .security_level(S_security_tone_level),
        .buzzer(O_buzzer));
    // Independent roles: K10 AWB, K8 application ACK, K5 ordinary tracking NEXT.
    assign S_btn_awb_evt=S_key_press[9];
    wire S_btn_ack_evt=S_key_press[7];
    assign S_btn_mode_evt=S_key_press[0];
    assign S_btn_detect_evt=S_key_press[1];
    assign S_btn_application_evt=S_key_press[2]; // K3: independent application toggle.
    assign S_btn_cal_evt=S_key_press[5];
    assign S_btn_ae_evt=S_key_press[6];
    // Local keypad is the sole control source.
    local_control_state u_control_state (
        .clk          (S_100m_clk),
        .rst_n        (S_rst_n),
        .btn_awb      (S_btn_awb_evt),
        .btn_ae       (S_btn_ae_evt),
        .btn_cal      (S_btn_cal_evt),
        .btn_detect   (S_btn_detect_evt),
        .btn_application (S_btn_application_evt),
        .btn_mode     (S_btn_mode_evt),
        .btn_ag_up(1'b0),.btn_ag_down(1'b0), // Analogue gain belongs to AE.
        .btn_dg_up(S_key_press[3]),.btn_dg_down(S_key_press[8]),
        .btn_defaults(1'b0),.current_again(7'd0),
        .restore_toggle(),
        .awb          (S_awb_enable),
        .ae           (S_ae_enable),
        .cal          (S_cal_enable),
        .detect       (S_detect_enable),
        .application  (S_application_enable),
        .mode         (S_mode),
        .manual_exposure    (S_manual_exposure),
        .manual_again       (S_manual_again),
        .manual_dgain       (S_manual_dgain),
        .manual_exposure_en (S_manual_exposure_en),
        .manual_again_en    (S_manual_again_en),
        .manual_dgain_en    (S_manual_dgain_en),
        .param_toggle       (S_param_toggle)
    );

    // Capture the manual values into the 24 MHz camera-config domain. Multi-bit
    // values cannot be plain 2-flop synchronized without risking a torn read,
    // so they are latched on the toggle edge after it has settled.
    always @(posedge S_24m_clk or negedge S_camera_cfg_rst_n) begin
        if(!S_camera_cfg_rst_n) begin
            S_param_toggle_sync_24m <= 3'b000;
            S_param_toggle_seen_24m <= 1'b0;
            S_ae_man_24m  <= 12'd1990;
            S_ag_man_24m  <= 7'd32;
            S_dg_man_24m  <= 7'd0;
            S_manual_en_24m <= 3'b000;
        end else begin
            S_param_toggle_sync_24m <= {S_param_toggle_sync_24m[1:0], S_param_toggle};
            if (S_param_toggle_sync_24m[2] ^ S_param_toggle_seen_24m) begin
                S_param_toggle_seen_24m <= S_param_toggle_sync_24m[2];
                S_ae_man_24m    <= S_manual_exposure;
                S_ag_man_24m    <= S_manual_again;
                S_dg_man_24m    <= S_manual_dgain;
                S_manual_en_24m <= {S_manual_dgain_en, S_manual_again_en,
                                    S_manual_exposure_en};
            end
        end
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw_seen <= 1'b0;
        else if(S_raw10_frame_start | S_axis_tvalid)
            S_dbg_raw_seen <= 1'b1;
    end

    // 从 RAW10 帧中均匀抽样亮度。统计在摄像头时钟域完成，结果每帧更新一次。
    auto_exposure_stats #(.IMG_WIDTH(CAM_WIDTH), .IMG_HEIGHT(CAM_HEIGHT)) u_auto_exposure_stats (
        .I_clk        (S_csi_rx_clk),
        .I_rst_n      (S_rst_n),
        .I_frame_start(S_raw_tuser),
        .I_valid      (S_raw_tvalid),
        .I_raw10      (S_raw_tdata),
        .O_luma_avg   (S_ae_luma_avg),
        .O_update_toggle(S_ae_luma_toggle)
    );

    // 原始 Bayer 统计直接取 image_correction 输出，尚未经过 demosaic/AWB。
    // 固定中央 512x256 ROI，输出均为 RAW10 十进制值（0..1023）。
    calibration_raw_stats #(.IMG_WIDTH(CAM_WIDTH), .IMG_HEIGHT(CAM_HEIGHT)) u_calibration_raw_stats (
        .I_clk           (S_csi_rx_clk),
        .I_rst_n         (S_rst_n),
        .I_tuser         (S_raw_tuser),
        .I_tlast         (S_raw_tlast),
        .I_tvalid        (S_raw_tvalid),
        .I_tdata         (S_raw_tdata),
        .O_b_avg         (S_cal_raw_b_avg),
        .O_gb_avg        (S_cal_raw_gb_avg),
        .O_gr_avg        (S_cal_raw_gr_avg),
        .O_r_avg         (S_cal_raw_r_avg),
        .O_clip_permille (S_cal_raw_clip_permille),
        .O_update_toggle (S_cal_raw_toggle)
    );

    // AE 状态和亮度统计跨到 24MHz 摄像头配置域。亮度总线在两次 toggle
    // 之间保持整帧稳定，因此两级采样后用 toggle 边沿产生单拍更新脉冲。
    always @(posedge S_24m_clk or negedge S_camera_cfg_rst_n) begin
        if(!S_camera_cfg_rst_n) begin
            S_ae_en_s0    <= 1'b0;
            S_ae_en_s1    <= 1'b0;
            S_ae_luma_s0  <= 8'd0;
            S_ae_luma_s1  <= 8'd0;
            S_ae_toggle_s0 <= 1'b0;
            S_ae_toggle_s1 <= 1'b0;
            S_ae_toggle_s2 <= 1'b0;
        end else begin
            S_ae_en_s0     <= S_ae_enable;
            S_ae_en_s1     <= S_ae_en_s0;
            S_ae_luma_s0   <= S_ae_luma_avg;
            S_ae_luma_s1   <= S_ae_luma_s0;
            S_ae_toggle_s0 <= S_ae_luma_toggle;
            S_ae_toggle_s1 <= S_ae_toggle_s0;
            S_ae_toggle_s2 <= S_ae_toggle_s1;
        end
    end

    // AWB / detection toggle states cross into the recovered CSI clock domain.
    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if (!S_rst_n) begin
            S_awb_en_s0 <= 1'b0;
            S_awb_en_s1 <= 1'b0;
            S_detect_en_s0 <= 1'b0;
            S_detect_en_s1 <= 1'b0;
        end else begin
            S_awb_en_s0 <= S_awb_enable;
            S_awb_en_s1 <= S_awb_en_s0;
            S_detect_en_s0 <= S_detect_enable;
            S_detect_en_s1 <= S_detect_en_s0;
        end
    end

    // 标定统计、曝光参数和三个开关状态跨到 65MHz HDMI 像素域。
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_cal_raw_bus_s0  <= 40'd0;
            S_cal_raw_bus_s1  <= 40'd0;
            S_cal_raw_tog_s0  <= 1'b0;
            S_cal_raw_tog_s1  <= 1'b0;
            S_cal_raw_tog_s2  <= 1'b0;
            S_cal_raw_bus_hdmi<= 40'd0;
            S_cal_rgb_bus_s0  <= 34'd0;
            S_cal_rgb_bus_s1  <= 34'd0;
            S_cal_rgb_tog_s0  <= 1'b0;
            S_cal_rgb_tog_s1  <= 1'b0;
            S_cal_rgb_tog_s2  <= 1'b0;
            S_cal_rgb_bus_hdmi<= 34'd0;
            S_cal_ae_s0       <= 16'd0;
            S_cal_ae_s1       <= 16'd0;
            S_cal_ag_s0       <= 16'd0;
            S_cal_ag_s1       <= 16'd0;
            S_cal_raw_clip_s0 <= 10'd0;
            S_cal_raw_clip_s1 <= 10'd0;
            S_cal_en_s0       <= 1'b0;
            S_cal_en_s1       <= 1'b0;
            S_cal_awb_s0      <= 1'b1;
            S_cal_awb_s1      <= 1'b1;
            S_cal_ae_en_s0    <= 1'b1;
            S_cal_ae_en_s1    <= 1'b1;
        end
        else begin
            S_cal_raw_bus_s0 <= {S_cal_raw_b_avg, S_cal_raw_gb_avg,
                                 S_cal_raw_gr_avg, S_cal_raw_r_avg};
            S_cal_raw_bus_s1 <= S_cal_raw_bus_s0;
            S_cal_raw_tog_s0 <= S_cal_raw_toggle;
            S_cal_raw_tog_s1 <= S_cal_raw_tog_s0;
            S_cal_raw_tog_s2 <= S_cal_raw_tog_s1;
            if(S_cal_raw_tog_s1 ^ S_cal_raw_tog_s2)
                S_cal_raw_bus_hdmi <= S_cal_raw_bus_s1;

            S_cal_rgb_bus_s0 <= {S_cal_rgb_r_avg, S_cal_rgb_g_avg,
                                 S_cal_rgb_b_avg, S_cal_clip_permille};
            S_cal_rgb_bus_s1 <= S_cal_rgb_bus_s0;
            S_cal_rgb_tog_s0 <= S_cal_rgb_toggle;
            S_cal_rgb_tog_s1 <= S_cal_rgb_tog_s0;
            S_cal_rgb_tog_s2 <= S_cal_rgb_tog_s1;
            if(S_cal_rgb_tog_s1 ^ S_cal_rgb_tog_s2)
                S_cal_rgb_bus_hdmi <= S_cal_rgb_bus_s1;

            S_cal_ae_s0    <= S_ae;
            S_cal_ae_s1    <= S_cal_ae_s0;
            S_cal_ag_s0    <= S_ag;
            S_cal_ag_s1    <= S_cal_ag_s0;
            S_cal_raw_clip_s0 <= S_cal_raw_clip_permille;
            S_cal_raw_clip_s1 <= S_cal_raw_clip_s0;
            S_cal_en_s0    <= S_cal_enable;
            S_cal_en_s1    <= S_cal_en_s0;
            S_cal_awb_s0   <= S_awb_enable;
            S_cal_awb_s1   <= S_cal_awb_s0;
            S_cal_ae_en_s0 <= S_ae_enable;
            S_cal_ae_en_s1 <= S_cal_ae_en_s0;
        end
    end

    wire [26:0] S_camera_param_levels;
    camera_parameter_levels u_camera_parameter_levels(
        .clk(S_hdmi_pixel_clk),.rst_n(S_hdmi_rst_n),.start(S_hdmi_user),
        .exposure(S_applied_exp_d1),.again(S_applied_gains_d1[31:16]),
        .dgain(S_applied_gains_d1[15:0]),.levels(S_camera_param_levels));

    calibration_bcd_formatter u_calibration_bcd_formatter (
        .I_clk             (S_hdmi_pixel_clk),
        .I_rst_n           (S_hdmi_rst_n),
        .I_start           (S_hdmi_user),
        .I_raw_b           (S_cal_raw_bus_hdmi[39:30]),
        .I_raw_gb          (S_cal_raw_bus_hdmi[29:20]),
        .I_raw_gr          (S_cal_raw_bus_hdmi[19:10]),
        .I_raw_r           (S_cal_raw_bus_hdmi[9:0]),
        .I_rgb_r           (S_cal_rgb_bus_hdmi[33:26]),
        .I_rgb_g           (S_cal_rgb_bus_hdmi[25:18]),
        .I_rgb_b           (S_cal_rgb_bus_hdmi[17:10]),
        .I_clip_permille   (S_cal_raw_clip_s1),
        .I_exposure        (S_applied_exp_d1),
        .I_gain            (S_again_milli),
        .I_dgain           (S_dgain_milli),
        .O_raw_b_bcd       (S_cal_raw_b_bcd),
        .O_raw_gb_bcd      (S_cal_raw_gb_bcd),
        .O_raw_gr_bcd      (S_cal_raw_gr_bcd),
        .O_raw_r_bcd       (S_cal_raw_r_bcd),
        .O_rgb_r_bcd       (S_cal_rgb_r_bcd),
        .O_rgb_g_bcd       (S_cal_rgb_g_bcd),
        .O_rgb_b_bcd       (S_cal_rgb_b_bcd),
        .O_clip_bcd        (S_cal_clip_bcd),
        .O_exposure_bcd    (S_cal_exposure_bcd),
        .O_gain_bcd        (S_cal_gain_bcd),
        .O_dgain_bcd       (S_cal_dgain_bcd)
    );

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_seen <= 1'b0;
        else if(S_ISP_O_tuser | S_ISP_O_tvalid)
            S_dbg_isp_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_wr_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_wr_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_rd_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_ddr_rd_seen <= 1'b1;
    end

    always @(posedge S_24m_clk or negedge S_camera_cfg_rst_n) begin
        if(!S_camera_cfg_rst_n)
            S_dbg_cam_cfg_seen <= 1'b0;
        else if(S_cam_cfg_done)
            S_dbg_cam_cfg_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_hs_seen <= 1'b0;
        else if(S_hs_rx_valid)
            S_dbg_hs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_csi_seen <= 1'b0;
        else if(S_csi_frame_start | S_csi_valid)
            S_dbg_csi_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw10_seen <= 1'b0;
        else if(S_raw10_frame_start | S_raw10_valid)
            S_dbg_raw10_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_fs_seen <= 1'b0;
        else if(S_ISP_O_tuser)
            S_dbg_isp_fs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_valid_seen <= 1'b0;
        else if(S_ISP_O_tvalid)
            S_dbg_isp_valid_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_req_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_req_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_ready_seen <= 1'b0;
        else if(S_ddr_user_ready)
            S_dbg_ddr_ready_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_req_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_req_seen <= 1'b0;
        else if(S_vo_ddr_rd_en)
            S_dbg_rd_req_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_valid_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_valid_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_rd_valid_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_nonzero_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_nonzero_seen <= 1'b0;
        else if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
            S_dbg_rd_nonzero_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_init_seen <= 1'b0;
        else if(S_init_calib_complete)
            S_dbg_mc_init_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_app_rdy_seen <= 1'b0;
        else if(S_mc_dbg_app_rdy)
            S_dbg_mc_app_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_wdf_rdy_seen <= 1'b0;
        else if(S_mc_dbg_wdf_rdy)
            S_dbg_mc_wdf_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_cmd_pop_seen <= 1'b0;
        else if(S_mc_dbg_cmd_pop)
            S_dbg_mc_cmd_pop_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_dbg_cam_cfg_seen_sync <= 2'b00;
            S_dbg_hs_seen_sync      <= 2'b00;
            S_dbg_csi_seen_sync     <= 2'b00;
            S_dbg_raw10_seen_sync   <= 2'b00;
            S_dbg_raw_seen_sync    <= 2'b00;
            S_dbg_isp_seen_sync    <= 2'b00;
            S_dbg_ddr_wr_seen_sync <= 2'b00;
            S_dbg_ddr_rd_seen_sync <= 2'b00;
            S_dbg_isp_fs_seen_sync  <= 2'b00;
            S_dbg_isp_valid_seen_sync <= 2'b00;
            S_dbg_ddr_req_seen_sync <= 2'b00;
            S_dbg_ddr_ready_seen_sync <= 2'b00;
            S_dbg_video_nonzero_seen <= 1'b0;
            S_dbg_rd_req_seen_sync <= 2'b00;
            S_dbg_rd_valid_seen_sync <= 2'b00;
            S_dbg_rd_nonzero_seen_sync <= 2'b00;
            S_dbg_mc_init_seen_sync <= 2'b00;
            S_dbg_mc_app_rdy_seen_sync <= 2'b00;
            S_dbg_mc_wdf_rdy_seen_sync <= 2'b00;
            S_dbg_mc_cmd_pop_seen_sync <= 2'b00;
        end
        else begin
            S_dbg_cam_cfg_seen_sync <= {S_dbg_cam_cfg_seen_sync[0], S_dbg_cam_cfg_seen};
            S_dbg_hs_seen_sync      <= {S_dbg_hs_seen_sync[0],      S_dbg_hs_seen};
            S_dbg_csi_seen_sync     <= {S_dbg_csi_seen_sync[0],     S_dbg_csi_seen};
            S_dbg_raw10_seen_sync   <= {S_dbg_raw10_seen_sync[0],   S_dbg_raw10_seen};
            S_dbg_raw_seen_sync    <= {S_dbg_raw_seen_sync[0],    S_dbg_raw_seen};
            S_dbg_isp_seen_sync    <= {S_dbg_isp_seen_sync[0],    S_dbg_isp_seen};
            S_dbg_ddr_wr_seen_sync <= {S_dbg_ddr_wr_seen_sync[0], S_dbg_ddr_wr_seen};
            S_dbg_ddr_rd_seen_sync <= {S_dbg_ddr_rd_seen_sync[0], S_dbg_ddr_rd_seen};
            S_dbg_isp_fs_seen_sync  <= {S_dbg_isp_fs_seen_sync[0],  S_dbg_isp_fs_seen};
            S_dbg_isp_valid_seen_sync <= {S_dbg_isp_valid_seen_sync[0], S_dbg_isp_valid_seen};
            S_dbg_ddr_req_seen_sync <= {S_dbg_ddr_req_seen_sync[0], S_dbg_ddr_req_seen};
            S_dbg_ddr_ready_seen_sync <= {S_dbg_ddr_ready_seen_sync[0], S_dbg_ddr_ready_seen};
            S_dbg_rd_req_seen_sync <= {S_dbg_rd_req_seen_sync[0], S_dbg_rd_req_seen};
            S_dbg_rd_valid_seen_sync <= {S_dbg_rd_valid_seen_sync[0], S_dbg_rd_valid_seen};
            S_dbg_rd_nonzero_seen_sync <= {S_dbg_rd_nonzero_seen_sync[0], S_dbg_rd_nonzero_seen};
            S_dbg_mc_init_seen_sync <= {S_dbg_mc_init_seen_sync[0], S_dbg_mc_init_seen};
            S_dbg_mc_app_rdy_seen_sync <= {S_dbg_mc_app_rdy_seen_sync[0], S_dbg_mc_app_rdy_seen};
            S_dbg_mc_wdf_rdy_seen_sync <= {S_dbg_mc_wdf_rdy_seen_sync[0], S_dbg_mc_wdf_rdy_seen};
            S_dbg_mc_cmd_pop_seen_sync <= {S_dbg_mc_cmd_pop_seen_sync[0], S_dbg_mc_cmd_pop_seen};
            if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
                S_dbg_video_nonzero_seen <= 1'b1;
        end
    end

    assign S_lane_error_any = |S_lane_error;

    assign S_vo_dbg_fifo_rst = 1'b0;
    assign S_vo_dbg_ddr_rd_valid = 1'b0;
    assign S_vo_dbg_fifo_rd_en = 1'b0;
    assign S_vo_dbg_fifo_empty = 1'b0;
    assign S_mc_dbg_fifo_afull = 1'b0;
    assign S_mc_dbg_app_rdy = 1'b0;
    assign S_mc_dbg_wdf_rdy = 1'b0;
    assign S_mc_dbg_cmd_pop = 1'b0;

    assign S_hdmi_debug_status = {
        (|S_ddr_user_rd_data),
        S_ddr_user_rd_valid,
        S_vo_ddr_rd_en,
        S_vi_ddr_wr_en
    };
	
	assign O_cam_rst = S_camera_cfg_rst_n;
    assign O_cam_24m 	= S_24m_clk;
	
	
	
	
    PLL u_PLL(
        .refclk      ( I_sys_clk         ),
        .reset    	 ( 1'b0              ),
        
        .clk0_out    ( S_100m_clk        ),
        .clk1_out    ( S_24m_clk         ),

        .clk4_out    (                   ),
        .clk5_out    (                   ),

        .lock        ( S_pll_lock        )

    );


    // Dedicated 74.25/371.25 MHz PLL; retain the camera's proven master clock.
    hdmi_720p_pll u_HDMI_PLL (
        .refclk(I_sys_clk), .reset(1'b0),
        .pixel_clk(S_hdmi_pixel_clk), .serial_clk(S_hdmi_serial_clk),
        .lock(S_hdmi_pll_lock)
    );

    //--------------------------------------------------------------------------
    // 按键轮询切换图像处理模式
    // 挂在 100MHz PLL 输出上：该时钟始终运行，而 S_csi_rx_clk 由摄像头
    // D-PHY 恢复，摄像头不出流时会停，按键会失效。
    //--------------------------------------------------------------------------

    // 跨到 MIPI 字节时钟域。mode 是多比特总线，两级同步器本身无法保证
    // 各比特同拍翻转，可能采到中间值（如 3->4 时瞬间出现 7）。
    // 由于源端已消抖、变化间隔远大于 10ms，这里再加一条"连续两拍相同才采纳"
    // 的过滤即可彻底消除瞬态值，无需格雷码。
    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if (!S_rst_n) begin
            S_mode_s0  <= 3'd0;
            S_mode_s1  <= 3'd0;
            S_mode_s2  <= 3'd0;
            S_mode_isp <= 3'd0;
        end else begin
            S_mode_s0 <= S_mode;
            S_mode_s1 <= S_mode_s0;
            S_mode_s2 <= S_mode_s1;
            if (S_mode_s1 == S_mode_s2)
                S_mode_isp <= S_mode_s2;
        end
    end
  ae_set u_ae_set (
      .I_clk(S_24m_clk),
      .I_rst(~S_camera_cfg_rst_n),
      .I_btn(4'b1111),
      .I_restore_defaults(1'b0),
      .I_auto_enable(S_ae_en_s1),
      .I_luma_avg(S_ae_luma_s1),
      .I_luma_valid(S_ae_luma_valid),
      .I_cam_cfg_done(S_cam_cfg_done),
      .I_ae_cfg_done(S_ae_cfg_done),
      // 本地按键手动接管：曝光/模拟增益为 [0]/[1]，数字增益单独一位。
      .I_manual_en      (S_manual_en_24m[1:0]),
      .I_manual_ae      ({4'b0, S_ae_man_24m}),
      // ae_set/uicfgcs500 use a legacy +16 bias; the local gain setting is LUT index 0..127.
      .I_manual_ag      ({9'b0, S_ag_man_24m} + 16'd16),
      .I_dg_manual_en   (S_manual_en_24m[2]),
      .I_manual_dg      ({9'b0, S_dg_man_24m}),
      .O_ae_req(S_ae_req),
      .O_ae(S_ae),
      .O_ag(S_ag),
      .O_dg(S_dg)
  );
  
    uicfgcs500 #(
      .CLK_DIV(24000000 / 100000 - 1),
      .CLK_FREQ_HZ(24000000),
      .IMG_WIDTH(CAM_WIDTH), .IMG_HEIGHT(CAM_HEIGHT)
  ) u_uicfgcs500 (
      .I_clk(S_24m_clk),  //系统时钟输入
      .I_rst_n(S_camera_cfg_rst_n),  // camera clock synchronous release
      .I_ae_req(S_ae_req),
      .I_ae(S_ae),
      .I_ag(S_ag),
      .I_dg(S_dg),
      .O_applied_params(S_camera_applied),
      .O_cam_scl(O_cam_scl),  //I2C总线，SCL时钟
      .IO_cam_sda(IO_cam_sda),  //I2C总线，SDA数据
      .O_cfg_done(S_cam_cfg_done),  //摄像头寄存器初始化完成
      .O_ae_cfg_done(S_ae_cfg_done), // ACK completion, not sensor readback
      .O_cfg_error(S_cam_cfg_error)
  );
	
    mipi_dphy_rx_ph1p_mipiio_wrapper#(
        .DPHY_RX_LOCATION      ( "DPHY0" ),
        .HS_EQUALIZER          ( "0dB"   ),//"0dB","1.5dB","3dB","4.5dB","6dB","7.5dB","9dB","10.5dB"
        .HS_VGA_GAIN           ( "8dB" ),//"-3dB","-1.5dB","0dB","1.5dB","3dB","4.5dB","6dB","7.5dB"
        .LANE_NUM              ( 2       ),
        .BYTE_NUM              ( 1       )
    )u_mipi_dphy_rx_ph1p_mipiio_wrapper(
        .I_lp_clk              ( S_100m_clk       ),
        .I_rst                 ( ~S_rst_n         ),

        .I_clk_lane_in_delay   ( 6'd0             ),
        .I_data_lane0_in_delay ( 6'd0             ),
        .I_data_lane1_in_delay ( 6'd0             ),
        .I_data_lane2_in_delay ( 6'd0             ),
        .I_data_lane3_in_delay ( 6'd0             ),

        .I_lane_invert         ( 4'b0000          ),
     
        .O_hs_rx_clk           ( S_csi_rx_clk     ),
        .O_hs_rx_valid         ( S_hs_rx_valid    ),
        .O_hs_rx_data          ( S_hs_rx_data     ),
      
        .O_lp_rx_lane0_p       (  ),
        .O_lp_rx_lane0_n       (  ),
      
        .I_lp_tx_en            ( 1'b0             ),
        .I_lp_tx_lane0_p       ( 1'b1             ),
        .I_lp_tx_lane0_n       ( 1'b1             ),
      
        .O_lane_match_error    (                  ),
        .O_lane_error          ( S_lane_error     ),
      
        .IO_rx_clk_pad_n       ( IO_rx_clk_pad_n  ),
        .IO_rx_clk_pad_p       ( IO_rx_clk_pad_p  ),
        .IO_rx_data_pad_n      ( IO_rx_data_pad_n ),
        .IO_rx_data_pad_p      ( IO_rx_data_pad_p )
    );
	
	
 //  cwc cwc_inst
 //(
 //    .probe0(S_hs_rx_valid),
 //    .probe1(S_hs_rx_data),
 //    .probe2(S_lane_error),
 //    .clk(S_csi_rx_clk)
 //);

 //csi 解码为RAW数据
csi_unpacket_2lane u_csi_unpacket(
.I_clk                 ( S_csi_rx_clk       ),
.I_rst_n               ( S_rst_n        	),
.I_hs_valid            ( S_hs_rx_valid     ),
.I_hs_data             ( S_hs_rx_data      ),

.O_csi_frame_start     ( S_csi_frame_start ),
.O_csi_frame_end       ( S_csi_frame_end   ),
.O_csi_valid           ( S_csi_valid       ),
.O_csi_data            ( S_csi_data        )
);


//解码为RAW8
raw10_unpacket_2lane u_raw10_unpacket (
.I_clk  (S_csi_rx_clk),
.I_rst_n(S_rst_n),

.I_csi_frame_start(S_csi_frame_start),
.I_csi_frame_end  (S_csi_frame_end),
.I_csi_valid      (S_csi_valid),
.I_csi_data       (S_csi_data),

.O_raw10_frame_start(S_raw10_frame_start),
.O_raw10_frame_end  (S_raw10_frame_end),
.O_raw10_valid      (S_raw10_valid),
.O_raw10_data       (S_raw10_data)
  );


//将数据转为stream流
uial2axis #(
.IMG_WIDTH(CAM_WIDTH),
.IMG_HEIGHT(CAM_HEIGHT),
.INPUT_DATA_WIDTH(40)
) 
u_uial2axis (
.I_native_clk(S_csi_rx_clk),
.I_rst_n     (S_rst_n),
.I_data      (S_raw10_data       ),
.I_data_valid(S_raw10_valid      ),
.I_data_start(S_raw10_frame_start),
.I_data_end  (S_raw10_frame_end  ),
.axis_tvalid (S_axis_tvalid),
.axis_tdata  (S_axis_tdata ),
.axis_tuser  (S_axis_tuser ),
.axis_tlast  (S_axis_tlast )
);
	

image_correction #(
    .DATA_WIDTH(40), .IMG_WIDTH(CAM_WIDTH), .IMG_HEIGHT(CAM_HEIGHT)
) u_image_correction (
    .I_clk   		(S_csi_rx_clk),
    .I_rst_n 		(S_rst_n),
					
    .I_raw_data 	(S_axis_tdata),
    .I_raw_frame_end 	(S_axis_tlast),
    .I_raw_valid	(S_axis_tvalid),
    .I_raw_frame_start 	(S_axis_tuser),
					
    .O_raw_tdata 	(S_raw_tdata ),
    .O_raw_tlast 	(S_raw_tlast ),
    .O_raw_tvalid	(S_raw_tvalid),
    .O_raw_tuser 	(S_raw_tuser ),
    .O_raw_tready   (1'b1)
);


//cwc1 cwc1_Inst
//  (
//      .probe0(S_axis_tuser),
//      .probe1(S_axis_tvalid),
//      .probe2(S_axis_tlast),
//      .probe3(S_axis_tdata),
//      .probe4(S_ISP_O_tuser),
//      .probe5(S_ISP_O_tvalid),
//      .probe6(S_ISP_O_tlast),
//      .probe7(S_ISP_O_tdata),
//      .probe8 (S_raw_tdata ),
//      .probe9 (S_raw_tlast ),
//      .probe10(S_raw_tvalid),
//      .probe11(S_raw_tuser ),
//      .clk(S_csi_rx_clk)
//  );


//ISP算法顶层模块
wire S_detect_result_strobe;
wire [438:0] S_detect_result;
wire [15:0] S_detect_capture_id;
wire S_detect_bad;
wire S_track_publish;
wire [495:0] S_track_frame_result;
wire [479:0] S_display_detect_track;
wire [15:0] S_servo_pulse_us;
wire S_detect_write_start, S_detect_read_start;
wire [1:0] S_detect_write_slot;
wire [422:0] S_detect_box;
wire S_display_latency_valid;
wire S_stat_new, S_stat_repeat;
wire [60:0] S_frame_statistics;
wire S_display_latency_overflow;
wire [3:0] S_display_latency_frames;
reg [1:0] S_detect_hdmi_sync;
always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
    if(!S_hdmi_rst_n) S_detect_hdmi_sync<=0;
    else S_detect_hdmi_sync<={S_detect_hdmi_sync[0],S_detect_enable};
end
// Application policy consumes detector metadata only; no image/frame store.
(* async_reg = "true" *) reg [1:0] S_application_camera_sync;
(* async_reg = "true" *) reg [1:0] S_application_hdmi_sync;
always @(posedge S_csi_rx_clk or negedge S_rst_n)
    if(!S_rst_n) S_application_camera_sync<=0;
    else S_application_camera_sync<={S_application_camera_sync[0],S_application_enable};
always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n)
    if(!S_hdmi_rst_n) S_application_hdmi_sync<=0;
    else S_application_hdmi_sync<={S_application_hdmi_sync[0],S_application_enable};
wire S_security_ack;
tracking_mailbox #(.WIDTH(1)) u_security_ack(
    .arst_n(S_rst_n && S_application_enable),.src_clk(S_100m_clk),
    .send(S_btn_ack_evt && !S_btn_detect_evt && !S_btn_application_evt && !S_btn_cal_evt && !S_btn_mode_evt),
    .data_in(1'b1),.dst_clk(S_csi_rx_clk),.receive(S_security_ack),.data_out());
wire [2:0] S_security_state,S_security_severity;
wire [46:0] S_security_seed;
wire S_security_track;
wire [15:0] S_security_red_count,S_security_blue_count,S_security_event_id;
wire [53:0] S_security_status={S_security_state,S_security_severity,
    S_security_red_count,S_security_blue_count,S_security_event_id};
wire [53:0] S_security_status_hdmi;
wire S_policy_snapshot_strobe,S_policy_snapshot_bad;
wire [15:0] S_policy_snapshot_id;
wire [422:0] S_policy_snapshot_boxes;
security_application #(.SHARED_BOXES(1),.IMG_WIDTH(CAM_WIDTH),.IMG_HEIGHT(CAM_HEIGHT)) u_security_application(
    .clk(S_csi_rx_clk),.rst_n(S_rst_n),.enable(S_application_camera_sync[1]),
    .ack(S_security_ack),.result_strobe(S_policy_snapshot_strobe),.result_bad(S_policy_snapshot_bad),
    .result_id(S_policy_snapshot_id),.capture_id(S_detect_capture_id),
    .boxes(S_policy_snapshot_boxes),.app_state(S_security_state),.severity(S_security_severity),
    .event_seed(S_security_seed),.track_enable(S_security_track),
    .red_count_bcd(S_security_red_count),.blue_count_bcd(S_security_blue_count),
    .event_source_id(S_security_event_id),.status_publish());
// Status is latest-state telemetry (event source ID is displayed explicitly),
// independent of the existing source-frame-matched box/video bridge.
detect_mailbox #(.WIDTH(54),.LOCAL_RESETS(1)) u_security_status_hdmi(
    .rst_n(S_rst_n),.src_clk(S_csi_rx_clk),.send(1'b1),.data_in(S_security_status),.ready(),
    .dst_clk(S_hdmi_pixel_clk),.receive(),.data_out(S_security_status_hdmi));
wire [2:0] S_security_camera_tone = S_application_camera_sync[1] &&
    (S_security_state==3'd1 || S_security_state==3'd2) ? S_security_severity : 3'd0;
wire [2:0] S_security_tone_snapshot;
detect_mailbox #(.WIDTH(3),.LOCAL_RESETS(1)) u_security_status_tone(
    .rst_n(S_rst_n),.src_clk(S_csi_rx_clk),.send(1'b1),.data_in(S_security_camera_tone),.ready(),
    .dst_clk(S_100m_clk),.receive(),.data_out(S_security_tone_snapshot));
// Local exit/reset silences security immediately, even if camera clocks stop.
assign S_security_tone_level = S_application_enable ? S_security_tone_snapshot : 3'd0;
tracking_system #(
    .SECURITY_SUPPORT(1),
    .IMG_WIDTH(CAM_WIDTH),.IMG_HEIGHT(CAM_HEIGHT),
    .MIN_US(834),.MAX_US(2166),.DIRECTION(-1),
    .TILT_SUPPORT(1),.TILT_MIN_US(1352),.TILT_MAX_US(1648),.TILT_DIRECTION(1)
) u_tracking_system(
    .sys_clk(I_sys_clk),.camera_clk(S_csi_rx_clk),.rst_n(S_rst_n),
    .enable(S_detect_enable),.next_button_n(I_keys_n[4]),
    .app_mode(S_application_camera_sync[1]),.app_track_enable(S_security_track),.app_seed(S_security_seed),
    .result_strobe(S_detect_result_strobe),.result_id(S_detect_result[438:423]),
    .capture_id(S_detect_capture_id),.result_bad(S_detect_bad),
    .boxes(S_detect_result[422:0]),.pan_pwm(O_servo_pan_pwm),
    .pulse_us(S_servo_pulse_us),.tilt_pwm(O_servo_tilt_pwm),.tilt_pulse_us(),
    .frame_publish(S_track_publish),
    .snapshot_strobe(S_policy_snapshot_strobe),.snapshot_bad(S_policy_snapshot_bad),
    .snapshot_id(S_policy_snapshot_id),.snapshot_boxes(S_policy_snapshot_boxes),
    .frame_result(S_track_frame_result));
// Physical J2 pins 1/2 confirmed by user: GPIOB0/C18, GPIOB1/D15.
assign S_detect_box=S_display_detect_track[422:0];
detect_frame_bridge #(.BOX_WIDTH(480)) u_detect_frame_bridge(
    .rst_n(S_rst_n),.camera_clk(S_csi_rx_clk),
    .result_strobe(S_track_publish),.result(S_track_frame_result),
    .ddr_clk(S_ddr_clk),.write_start(S_detect_write_start),.write_slot(S_detect_write_slot),
    .read_start(S_detect_read_start),.read_slot(S_video_out_rp),
    .hdmi_clk(S_hdmi_pixel_clk),.display_box(S_display_detect_track),
    .display_latency_valid(S_display_latency_valid),
    .display_latency_overflow(S_display_latency_overflow),
    .display_latency_frames(S_display_latency_frames),
    .read_new_frame(S_stat_new),.read_repeat_frame(S_stat_repeat));
video_frame_statistics u_frame_statistics(
    .rst_n(S_rst_n),.sys_clk(I_sys_clk),.ddr_clk(S_ddr_clk),
    .camera_clk(S_csi_rx_clk),.hdmi_clk(S_hdmi_pixel_clk),
    .write_start(S_detect_write_start),.read_new_frame(S_stat_new),
    .read_repeat_frame(S_stat_repeat),.detect_result(S_detect_result_strobe),
    .hdmi_frame(S_hdmi_de && S_hdmi_user),.display_stats(S_frame_statistics));
isp_top #(.IMG_WIDTH(CAM_WIDTH), .IMG_HEIGHT(CAM_HEIGHT), .DETECT_SUPPORT(1)) u_isp_top (
.axi4s_video_aclk(S_csi_rx_clk),
.I_rst_n         (S_rst_n),
.I_mode          (S_mode_isp),
.I_awb_enable    (S_awb_en_s1),
.I_detect_switch_n(~S_detect_en_s1),
.O_detect_result_strobe(S_detect_result_strobe),
.O_detect_result(S_detect_result),
.O_detect_capture_id(S_detect_capture_id),
.O_detect_bad(S_detect_bad),
.I_tlast         (S_raw_tlast	),
.I_tuser         (S_raw_tuser	),
.I_tdata         (S_raw_tdata	),
.I_tvalid        (S_raw_tvalid	),
.I_tdest         (),
.O_tready        (1'b1),              // video_in 无反压接口，ISP 下游始终可接收
.O_tdata         (S_ISP_O_tdata ),
.O_tlast         (S_ISP_O_tlast ),
.O_tuser         (S_ISP_O_tuser ),
.O_tvalid        (S_ISP_O_tvalid),
.O_eth_tdata     (),
.O_eth_tuser     (),
.O_eth_tvalid    (),
.I_tready        (),
.O_cal_r_avg     (S_cal_rgb_r_avg),
.O_cal_g_avg     (S_cal_rgb_g_avg),
.O_cal_b_avg     (S_cal_rgb_b_avg),
.O_cal_clip_permille(S_cal_clip_permille),
.O_cal_update_toggle(S_cal_rgb_toggle)
  );

    // Local-only design: keep the unused Ethernet PHY in reset.
    assign O_eth_phy_reset_n = 1'b0;

    video_in u_video_in(
        .I_rst_n              (  S_rst_n             ),

        .I_camera_clk         ( S_csi_rx_clk          ),
        .I_camera_frame_start ( S_ISP_O_tuser  ),
        .I_camera_valid       ( S_ISP_O_tvalid ),
        .I_camera_data        ( S_ISP_O_tdata  ),
        .I_mipi_rx_error      ( 1'b0			      ),

        .I_ddr_clk            ( S_ddr_clk             ),
		.I_display_pause      ( 1'b0                  ),
        .I_video_out_rd_busy  ( S_video_out_rd_busy ),
        .O_video_in_wr_busy   ( S_video_in_wr_busy    ),
        .O_video_out_rp       ( S_video_out_rp        ),
        .O_write_frame_start ( S_detect_write_start  ),
        .O_write_frame_slot  ( S_detect_write_slot   ),

        .O_ddr_user_wr_en     ( S_vi_ddr_wr_en        ),
        .O_ddr_user_addr      ( S_vi_ddr_wr_addr      ),
        .O_ddr_user_wr_data   ( S_vi_ddr_wr_data      ),
        .I_ddr_user_ready     ( S_ddr_user_ready      )
    );



    assign S_ddr_user_wr_en    = S_vi_ddr_wr_en;

    assign S_ddr_user_rd_en    = S_vo_ddr_rd_en;

    assign S_ddr_user_addr     = S_vi_ddr_wr_en ? S_vi_ddr_wr_addr :
                                 S_vo_ddr_rd_en ? S_vo_ddr_rd_addr : 'd0;

    assign S_ddr_user_wr_data  = S_vi_ddr_wr_data;

    assign S_video_out_rst_n = S_hdmi_rst_n;

    // Sticky activity indicators remain visible on the details page even if
    // the camera/DDR path stops; all flags cross into the HDMI domain here.
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_link_status_meta <= 8'd0;
            S_link_status_sync <= 8'd0;
        end else begin
            S_link_status_meta <= {S_init_calib_complete, S_dbg_ddr_ready_seen,
                                   S_dbg_ddr_wr_seen, S_dbg_ddr_rd_seen,
                                   4'b0000};
            S_link_status_sync <= S_link_status_meta;
        end
    end

    video_out u_video_out(
        .I_rst_n             ( S_video_out_rst_n    ),
        .I_ddr_clk           ( S_ddr_clk            ),
 
        .O_video_out_rd_busy ( S_video_out_rd_busy  ),
        .I_video_in_wr_busy  ( S_video_in_wr_busy ),
        .I_video_out_rp      ( S_video_out_rp       ),
        .O_read_frame_start ( S_detect_read_start  ),
 
        .O_ddr_user_rd_en    ( S_vo_ddr_rd_en       ),
        .O_ddr_user_addr     ( S_vo_ddr_rd_addr     ),
        .I_ddr_user_ready    ( S_ddr_user_ready     ),
        .I_ddr_user_rd_valid ( S_ddr_user_rd_valid ),
        .I_ddr_user_rd_data  ( S_ddr_user_rd_data   ),

        .I_dsi_clk           ( S_hdmi_pixel_clk     ),
        .I_video_vsync       ( S_video_out_vsync    ),
        .I_video_rd_en       ( S_hdmi_window_rd_en  ),
        .O_vdieo_data        ( S_video_rd_data      )
    );



    mc_to_user_interface u_mc_to_user_interface(
        .I_clk                   ( S_ddr_clk               ),
        .I_rst_n                 ( S_rst_n                 ),

        .I_ddr_user_wr_en        ( S_ddr_user_wr_en        ),
        .I_ddr_user_rd_en        ( S_ddr_user_rd_en        ),
        .I_ddr_user_addr         ( S_ddr_user_addr         ),
        .I_ddr_user_wr_data      ( S_ddr_user_wr_data      ),
        .O_ddr_user_ready        ( S_ddr_user_ready        ),
        .O_ddr_user_rd_valid     ( S_ddr_user_rd_valid     ),
        .O_ddr_user_rd_data      ( S_ddr_user_rd_data      ),
            
        .O_mc_app_en             ( S_mc_app_en             ),
        .O_mc_app_addr           ( S_mc_app_addr           ),
        .O_mc_app_cmd            ( S_mc_app_cmd            ),
        .I_mc_app_rdy            ( S_mc_app_rdy            ),
        .O_mc_app_wdf_wren       ( S_mc_app_wdf_wren       ),
        .O_mc_app_wdf_data       ( S_mc_app_wdf_data       ),
        .O_mc_app_wdf_end        ( S_mc_app_wdf_end        ),
        .O_mc_app_wdf_mask       ( S_mc_app_wdf_mask       ),
        .I_mc_app_wdf_rdy        ( S_mc_app_wdf_rdy        ),
        .I_mc_app_rd_data        ( S_mc_app_rd_data        ),
        .I_mc_app_rd_data_end    ( S_mc_app_rd_data_end    ),
        .I_mc_app_rd_data_valid  ( S_mc_app_rd_data_valid  )
    );



    ph1p35_324_ddr_wrapper u_ph1p35_324_ddr_wrapper(
        .I_sys_clk              ( I_sys_clk              ),
        .I_sys_rst_n            ( S_rst_n                ),

        .O_ddr_clk              ( S_ddr_clk              ),
        .O_init_calib_complete  ( S_init_calib_complete  ),
        .I_mc_app_addr          ( S_mc_app_addr          ),
        .I_mc_app_cmd           ( S_mc_app_cmd           ),
        .I_mc_app_en            ( S_mc_app_en            ),
        .I_mc_app_wdf_data      ( S_mc_app_wdf_data      ),
        .I_mc_app_wdf_end       ( S_mc_app_wdf_end       ),
        .I_mc_app_wdf_mask      ( S_mc_app_wdf_mask      ),
        .I_mc_app_wdf_wren      ( S_mc_app_wdf_wren      ),
        .O_mc_app_rd_data       ( S_mc_app_rd_data       ),
        .O_mc_app_rd_data_end   ( S_mc_app_rd_data_end   ),
        .O_mc_app_rd_data_valid ( S_mc_app_rd_data_valid ),
        .O_mc_app_rdy           ( S_mc_app_rdy           ),
        .O_mc_app_wdf_rdy       ( S_mc_app_wdf_rdy       ),

        .ddr_addr               ( ddr_addr               ),
        .ddr_ba                 ( ddr_ba                 ),
        .ddr_cke                ( ddr_cke                ),
        .ddr_odt                ( ddr_odt                ),
        .ddr_cs_n               ( ddr_cs_n               ),
        .ddr_ras_n              ( ddr_ras_n              ),
        .ddr_cas_n              ( ddr_cas_n              ),
        .ddr_we_n               ( ddr_we_n               ),
        .ddr_ck_p               ( ddr_ck_p               ),
        .ddr_ck_n               ( ddr_ck_n               ),
        .ddr_dm                 ( ddr_dm                 ),
        .ddr_dq                 ( ddr_dq                 ),
        .ddr_dqs_p              ( ddr_dqs_p              ),
        .ddr_dqs_n              ( ddr_dqs_n              )
    );



    // 1280x720p60: 1650x750 totals, 74.25 MHz pixel clock. No black bar.
    uivtc #(
        .H_ActiveSize ( CAM_WIDTH ),
        .H_FrameSize  ( 1650 ),
        .H_SyncStart  ( 1390 ),
        .H_SyncEnd    ( 1430 ),
        .V_ActiveSize ( CAM_HEIGHT ),
        .V_FrameSize  ( 750  ),
        .V_SyncStart  ( 725  ),
        .V_SyncEnd    ( 730  )
    )u_hdmi_vtc(
        .I_vtc_rstn    ( S_hdmi_rst_n    ),
        .I_vtc_clk     ( S_hdmi_pixel_clk ),
        .O_vtc_vs      ( S_hdmi_vsync     ),
        .O_vtc_hs      ( S_hdmi_hsync     ),
        .O_vtc_de_valid( S_hdmi_de        ),
        .O_vtc_user    ( S_hdmi_user      ),
        .O_vtc_last    ( S_hdmi_last      )
    );

    // Stable status bits into the HDMI clock domain. No video timing changes.
    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n) S_isp_nonzero_seen <= 1'b0;
        else if(S_ISP_O_tvalid && (|S_ISP_O_tdata)) S_isp_nonzero_seen <= 1'b1;
    end
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_cam_status_meta <= 3'b000;
            S_cam_status_hdmi <= 3'b000;
            S_mode_hdmi_meta <= 3'd0;
            S_mode_hdmi_sync <= 3'd0;
            S_mode_hdmi_prev <= 3'd0;
            S_mode_hdmi <= 3'd0;
            S_isp_nonzero_sync <= 2'b00;
        end else begin
            S_cam_status_meta <= {S_cam_cfg_error,S_cam_cfg_done,S_ae_cfg_done};
            S_cam_status_hdmi <= S_cam_status_meta;
            // Display the mode actually accepted by the ISP. CSI -> HDMI is
            // already an asynchronous clock boundary in the project SDC.
            // Two-stage synchronizer plus stable-value qualification prevents
            // a transient multi-bit mode code from reaching the OSD.
            S_mode_hdmi_meta <= S_mode_isp;
            S_mode_hdmi_sync <= S_mode_hdmi_meta;
            S_mode_hdmi_prev <= S_mode_hdmi_sync;
            if(S_mode_hdmi_sync == S_mode_hdmi_prev)
                S_mode_hdmi <= S_mode_hdmi_sync;
            S_isp_nonzero_sync <= {S_isp_nonzero_sync[0],S_isp_nonzero_seen};
        end
    end

    hdmi_mixer #(
        .H_OFFSET   ( 0    ),
        .V_OFFSET   ( 0    ),  // Full 720p video; OSD stays at its old coordinates.
        .IMG_WIDTH  ( CAM_WIDTH ),
        .IMG_HEIGHT ( CAM_HEIGHT ),
        .DEBUG_MODE ( 0    ), // 彩条测试通过，切回真实摄像头/ISP/DDR数据通路
        .DETECT_SUPPORT(1),.TRACKING_SUPPORT(1),.SECURITY_SUPPORT(1)
    )u_hdmi_mixer(
        .I_clk           ( S_hdmi_pixel_clk   ),
        .I_rst_n         ( S_hdmi_rst_n       ),
        .I_video_vsync   ( S_hdmi_vsync       ),
        .I_video_hsync   ( S_hdmi_hsync       ),
        .I_video_de      ( S_hdmi_de          ),
        .I_video_user    ( S_hdmi_user        ),
        .I_video_last    ( S_hdmi_last        ),
        .I_debug_status  ( S_hdmi_debug_status),
        .I_cal_enable    ( S_cal_en_s1 && !S_detect_hdmi_sync[1] ),
        .I_notice_detail_enable( S_cal_en_s1 ),
        .I_notice_detect_enable( S_detect_hdmi_sync[1] ),
        .I_link_status   ( S_link_status_sync ),
        .I_detect_box    ( S_detect_box       ),
        .I_track_box(S_display_detect_track[469:423]),
        .I_track_id(S_display_detect_track[477:470]),
        .I_track_state(S_display_detect_track[479:478]),
        .I_security_enable(S_application_hdmi_sync[1]),
        .I_security_status(S_security_status_hdmi),
        .I_frame_statistics(S_frame_statistics),
        .I_latency_valid ( S_display_latency_valid ),
        .I_latency_overflow(S_display_latency_overflow),
        .I_latency_frames(S_display_latency_frames),
        .I_cam_cfg_error ( S_cam_status_hdmi[2]),
        .I_cam_cfg_ready ( &S_cam_status_hdmi[1:0]),
        .I_cal_mode      ( S_mode_hdmi ),
        .I_cal_path_status({S_dbg_raw_seen_sync[1],S_isp_nonzero_sync[1],
                            S_dbg_ddr_wr_seen_sync[1],S_dbg_video_nonzero_seen}),
        .I_cal_awb_enable( S_cal_awb_s1       ),
        .I_cal_ae_enable ( S_cal_ae_en_s1     ),
        .I_cal_raw_b_bcd ( S_cal_raw_b_bcd    ),
        .I_cal_raw_gb_bcd( S_cal_raw_gb_bcd   ),
        .I_cal_raw_gr_bcd( S_cal_raw_gr_bcd   ),
        .I_cal_raw_r_bcd ( S_cal_raw_r_bcd    ),
        .I_cal_rgb_r_bcd ( S_cal_rgb_r_bcd    ),
        .I_cal_rgb_g_bcd ( S_cal_rgb_g_bcd    ),
        .I_cal_rgb_b_bcd ( S_cal_rgb_b_bcd    ),
        .I_cal_clip_bcd  ( S_cal_clip_bcd     ),
        .I_cal_exposure_bcd(S_cal_exposure_bcd),
        .I_cal_gain_bcd  ( S_cal_gain_bcd     ),
        .I_cal_dgain_bcd ( S_cal_dgain_bcd ),
        .I_cal_params_valid(S_camera_applied_hdmi[63:48]==16'hCA01),
        .I_cal_param_levels(S_camera_param_levels),
        .O_video_rd_en   ( S_hdmi_window_rd_en),
        .I_video_rd_data ( S_video_rd_data    ),
        .O_hdmi_vsync    ( S_hdmi_out_vsync   ),
        .O_hdmi_hsync    ( S_hdmi_out_hsync   ),
        .O_hdmi_de       ( S_hdmi_out_de      ),
        .O_hdmi_data     ( S_hdmi_out_data    )
    );

    hdmi_tx u_hdmi_tx(
        .I_pixel_clk        ( S_hdmi_pixel_clk  ),
        .I_serial_clk       ( S_hdmi_serial_clk ),
        .I_rst              ( ~S_hdmi_rst_n     ),
        .I_key_in           ( 1'b0              ),
        .I_edid_read_trig   ( 1'b0              ),
        .O_edid_read_valid  (                   ),
        .O_edid_read_data   (                   ),
        .I_video_rgb_enable ( 1'b1              ),
        .I_video_in_vs      ( S_hdmi_out_vsync  ),
        .I_video_in_de      ( S_hdmi_out_de     ),
        .I_video_in_user    ( 1'b0              ),
        .I_video_in_valid   ( 1'b0              ),
        .I_video_in_last    ( 1'b0              ),
        .O_video_in_ready   (                   ),
        .I_video_in_data    ( S_hdmi_out_data   ),
        .I_audio_valid      ( 1'b0              ),
        .I_audio_left_data  ( 24'd0             ),
        .I_audio_right_data ( 24'd0             ),
        .I_i2s_BCLK         ( 1'b0              ),
        .I_i2s_LRCK         ( 1'b0              ),
        .I_i2s_DOUT         ( 1'b0              ),
        .O_ddc_scl          (                   ),
        .O_hdmi_clk_p       (                   ),
        .O_hdmi_tx_p        (                   ),
        .O_tmds_ch0_p       ( O_tmds_ch0_p      ),
        .O_tmds_ch1_p       ( O_tmds_ch1_p      ),
        .O_tmds_ch2_p       ( O_tmds_ch2_p      ),
        .O_tmds_clk_p       ( O_tmds_clk_p      )
    );


    
endmodule
