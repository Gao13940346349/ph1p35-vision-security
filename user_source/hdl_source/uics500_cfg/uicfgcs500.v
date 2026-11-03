
/*******************************MILIANKE*******************************
*Company : MiLianKe Electronic Technology Co., Ltd.
*WebSite:https://www.milianke.com
*TechWeb:https://www.uisrc.com
*tmall-shop:https://milianke.tmall.com
*jd-shop:https://milianke.jd.com
*taobao-shop1: https://milianke.taobao.com
*Create Date: 2019/12/17
*Module Name:uicfg7611
*File Name:uicfg7611.v
*Description: 
*The reference demo provided by Milianke is only used for learning. 
*We cannot ensure that the demo itself is free of bugs, so users 
*should be responsible for the technical problems and consequences
*caused by the use of their own products.
*Copyright: Copyright (c) MiLianKe
*All rights reserved.
*Revision: 1.0
*Signal description
*1) I_ input
*2) O_ output
*3) IO_ input output
*4) S_ system internal signal
*5) _n activ low
*6) _dg debug signal 
*7) _r delay or register
*8) _s state mechine
*********************************************************************/

/*******************************uicfg7611***************************
--1.ADV7611芯片配置驱动程序
*********************************************************************/

`timescale 1ns / 1ns

module uicfgcs500 #(
    parameter integer CLK_DIV = 239,
    parameter integer CLK_FREQ_HZ = 24000000,
    parameter integer IMG_WIDTH = 1024,
    parameter integer IMG_HEIGHT = 600
)(
    input wire I_clk,
    input wire I_rst_n,
    input wire I_ae_req,
    input wire [15:0] I_ae,
    input wire [15:0] I_ag,
    input wire [15:0] I_dg,
    output wire O_cam_scl,
    inout wire IO_cam_sda,
    output reg O_cfg_done,
    output reg O_ae_cfg_done,
    output reg O_cfg_error,
    output reg [63:0] O_applied_params
);
    localparam [7:0] CAM_ID = 8'h6c;
    localparam [2:0] SELECT_REG=0, LAUNCH=1, WAIT_BUSY=2, WAIT_DONE=3,
                     IDLE=4, GAIN_WAIT=5, GAIN_LATCH=6, FAILED=7;
    // A write/read takes fewer than 50 SCL periods. Never wait indefinitely.
    localparam integer TIMEOUT_CYCLES = (CLK_DIV+1)*128;
    localparam integer POST_STREAM_DELAY_CYCLES = CLK_FREQ_HZ/100; // 10 ms
    localparam [15:0] AE_MAX = 16'd1990; // 2*VTS-10 for VTS=1000

    reg [7:0] rst_cnt = 0;
    always @(posedge I_clk or negedge I_rst_n)
        if(!I_rst_n) rst_cnt <= 0;
        else if(!rst_cnt[7]) rst_cnt <= rst_cnt + 1'b1;

    wire iic_busy, iic_error;
    wire [7:0] rd_data;
    reg busy_meta, busy_sync, error_meta, error_sync;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            busy_meta<=0; busy_sync<=0; error_meta<=0; error_sync<=0;
        end else begin
            busy_meta<=iic_busy; busy_sync<=busy_meta;
            error_meta<=iic_error; error_sync<=error_meta;
        end
    end

    reg [2:0] state;
    reg [8:0] reg_index;
    wire [8:0] init_size, ae_size;
    wire [31:0] init_data, ae_data;
    wire [23:0] reg_data = O_cfg_done ? ae_data[23:0] : init_data[23:0];
    wire [8:0] reg_size = O_cfg_done ? ae_size : init_size;
    reg iic_req;
    reg [31:0] wr_data;
    reg read_mode;
    reg transaction_error;
    reg [1:0] retry_count;
    reg [31:0] timeout_count;
    reg [31:0] post_stream_delay_count;

    // Only 3e03[3:0] is documented. Read/modify/write preserves its upper bits.
    reg mode_read_done;
    reg [3:0] mode_upper;
    wire needs_mode_read = !O_cfg_done && reg_data[23:8]==16'h3e03;
    // Factory 720p60 requires 10 ms between stream-on and the final 0x302d.
    wire needs_post_stream_delay = !O_cfg_done &&
                                   (reg_index == init_size-1'b1) &&
                                   (init_data[23:8] == 16'h302d);

    // One-entry latest-request queue. Active values never change mid-group.
    reg pending;
    reg [15:0] pending_ae;
    reg [15:0] pending_dg;
    reg [6:0] pending_gain_index;
    reg [15:0] active_ae, active_gain, active_dg;
    reg [6:0] active_gain_index;
    wire [15:0] gain_value;
    // Legacy register selector = LUT index + 16. 16..143 covers all 128
    // manual gain steps; ae_set retains its separate automatic upper limit.
    wire [15:0] bounded_ag = I_ag < 16 ? 16'd16 :
                                  I_ag > 143 ? 16'd143 : I_ag;
    wire [15:0] bounded_ae = I_ae < 64 ? 16'd64 :
                                  I_ae > AE_MAX ? AE_MAX : I_ae;
    wire [15:0] gain_offset = bounded_ag - 16'd16;

    always @(posedge I_clk) begin
        if(!rst_cnt[7]) begin
            state<=SELECT_REG; reg_index<=0;
            O_cfg_done<=0; O_ae_cfg_done<=0; O_cfg_error<=0;
            O_applied_params<=64'd0;
            iic_req<=0; wr_data<=0; read_mode<=0;
            transaction_error<=0; retry_count<=0; timeout_count<=0;
            post_stream_delay_count<=0;
            mode_read_done<=0; mode_upper<=0;
            pending<=0; pending_ae<=AE_MAX; pending_gain_index<=0; pending_dg<=16'h0080;
            active_ae<=AE_MAX; active_gain<=16'h0320; active_gain_index<=0; active_dg<=16'h0080;
        end else begin
            if(I_ae_req) begin
                pending<=1;
                pending_ae<=bounded_ae;
                pending_dg<=I_dg;
                pending_gain_index<=gain_offset[6:0];
            end
            if((state==WAIT_BUSY || state==WAIT_DONE) && error_sync)
                transaction_error<=1;

            if(state==LAUNCH || state==WAIT_BUSY || state==WAIT_DONE)
                timeout_count<=timeout_count+1'b1;
            else
                timeout_count<=0;

            if((state==LAUNCH || state==WAIT_BUSY || state==WAIT_DONE) &&
               timeout_count >= TIMEOUT_CYCLES) begin
                iic_req<=0;
                O_cfg_error<=1;
                O_ae_cfg_done<=0;
                state<=FAILED;
            end else case(state)
                SELECT_REG: begin
                    if(needs_post_stream_delay &&
                       post_stream_delay_count < POST_STREAM_DELAY_CYCLES) begin
                        post_stream_delay_count <= post_stream_delay_count + 1'b1;
                    end else if(reg_index >= reg_size) begin
                        // Reset the index WHEN changing tables; never wrap
                        // through out-of-range address 0000 after init.
                        reg_index<=0;
                        O_cfg_done<=1;
                        O_ae_cfg_done<=1;
                        // Publish only after every write in the group was ACKed.
                        // This reports committed writes, not sensor readback.
                        if (O_cfg_done)
                            O_applied_params <= {16'hCA01, active_ae,
                                                active_gain, active_dg};
                        state<=IDLE;
                    end else state<=LAUNCH;
                end
                LAUNCH: if(!busy_sync) begin
                    read_mode<=needs_mode_read && !mode_read_done;
                    wr_data[7:0]<=CAM_ID;
                    wr_data[15:8]<=reg_data[23:16];
                    wr_data[23:16]<=reg_data[15:8];
                    wr_data[31:24]<=needs_mode_read ?
                                      {mode_upper,4'hb} : reg_data[7:0];
                    transaction_error<=0;
                    iic_req<=1;
                    state<=WAIT_BUSY;
                end
                WAIT_BUSY: if(busy_sync) begin
                    iic_req<=0;
                    state<=WAIT_DONE;
                end
                WAIT_DONE: if(!busy_sync) begin
                    if(transaction_error || error_sync) begin
                        if(retry_count==2) begin
                            O_cfg_error<=1;
                            O_ae_cfg_done<=0;
                            state<=FAILED;
                        end else begin
                            retry_count<=retry_count+1'b1;
                            state<=SELECT_REG;
                        end
                    end else begin
                        retry_count<=0;
                        if(read_mode) begin
                            mode_upper<=rd_data[7:4];
                            mode_read_done<=1;
                        end else begin
                            reg_index<=reg_index+1'b1;
                            mode_read_done<=0;
                        end
                        state<=SELECT_REG;
                    end
                end
                IDLE: if(pending && !I_ae_req) begin
                    active_ae<=pending_ae;
                    active_dg<=pending_dg;
                    active_gain_index<=pending_gain_index;
                    pending<=0;
                    reg_index<=0;
                    O_ae_cfg_done<=0;
                    state<=GAIN_WAIT;
                end
                GAIN_WAIT: state<=GAIN_LATCH; // synchronous gain LUT
                GAIN_LATCH: begin
                    active_gain<=gain_value;
                    state<=SELECT_REG;
                end
                FAILED: iic_req<=0; // Require board reset after persistent bus error.
                default: begin
                    O_cfg_error<=1; O_ae_cfg_done<=0; iic_req<=0; state<=FAILED;
                end
            endcase
        end
    end

    uii2c #(.WMEN_LEN(4), .RMEN_LEN(1), .CLK_DIV(CLK_DIV)) uii2c_inst (
        .I_clk(I_clk), .I_rstn(rst_cnt[7]),
        .O_iic_scl(O_cam_scl), .IO_iic_sda(IO_cam_sda),
        .I_wr_data(wr_data), .I_wr_cnt(read_mode ? 8'd3 : 8'd4),
        .O_rd_data(rd_data), .I_rd_cnt(read_mode ? 8'd1 : 8'd0),
        .I_iic_mode(read_mode), .I_iic_req(iic_req),
        .O_iic_busy(iic_busy), .O_iic_bus_error(iic_error),
        .IO_iic_sda_dg()
    );
    uics500reg #(.IMG_WIDTH(IMG_WIDTH), .IMG_HEIGHT(IMG_HEIGHT)) init_table (
        .REG_SIZE(init_size), .REG_INDEX(reg_index), .REG_DATA(init_data)
    );
    SC500GainTbl gain_tbl (
        .I_clk(I_clk), .I_rstn(rst_cnt[7]),
        .I(active_gain_index), .O(gain_value)
    );
    uics500regAE ae_table (
        .REG_SIZE(ae_size), .REG_INDEX(reg_index), .REG_DATA(ae_data),
        .I_AG(active_gain), .I_AE(active_ae), .I_DG(active_dg)
    );
endmodule
