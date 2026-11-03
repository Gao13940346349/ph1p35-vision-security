`timescale 1ns / 1ps
/*******************************MILIANKE*******************************
*Company : MiLianKe Electronic Technology Co., Ltd.
*WebSite:https://www.milianke.com
*TechWeb:https://www.uisrc.com
*tmall-shop:https://milianke.tmall.com
*jd-shop:https://milianke.jd.com
*taobao-shop1: https://milianke.taobao.com
*Create Date: 2019/12/17
*Module Name:HDMI_IN_Test
*File Name:HDMI_IN_Test.v
*Description: 
*The reference demo provided by Milianke is only used for learning. 
*We cannot ensure that the demo itself is free of bugs, so users 
*should be responsible for the technical problems and consequences
*caused by the use of their own products.
*Copyright: Copyright (c) MiLianKe
*All rights reserved.
*Revision: 1.0
*Signal description
*1) _i input
*2) _o output
*3) _n activ low
*4) _dg debug signal 
*5) _r delay or register
*6) _s state mechine
*********************************************************************/

/*******************************ui7611reg***************************
--1.ADV7611芯片配置寄存器配置表
*********************************************************************/

module uics500regAE
(
input      [8 :0]  REG_INDEX,
output reg [31:0]  REG_DATA,
output     [8 :0]  REG_SIZE,
input      [15:0]  I_AE,
input      [15:0]  I_AG,
input      [15:0]  I_DG
);

// Group 0: atomically latch exposure, analogue gain and digital gain at a frame
// boundary. 0x3817 delay nibble remains its reset value 0 (SC500CS manual
// section 2.3).
assign	REG_SIZE = 9'd10;
//-----------------------------------------------------------------
/////////////////////	Config Data REG	  //////////////////////////
always@(*)
   case(REG_INDEX)
		0:	REG_DATA = {16'h3800, 8'h00};
		1:	REG_DATA = {16'h3e00, 4'h0, I_AE[15:12]};
		2:	REG_DATA = {16'h3e01, I_AE[11: 4]};
		3:	REG_DATA = {16'h3e02, I_AE[3:0], 4'h0};
		4:	REG_DATA = {16'h3e08, I_AG[15:8]};
		5:	REG_DATA = {16'h3e09, I_AG[7 :0]};
		// 数字增益与模拟增益同组生效，避免两者在不同帧落地造成中间态。
		6:	REG_DATA = {16'h3e06, I_DG[15:8]};
		7:	REG_DATA = {16'h3e07, I_DG[7 :0]};
		8:	REG_DATA = {16'h3800, 8'h10};
		9:	REG_DATA = {16'h3800, 8'h40};
		default:REG_DATA    =   {16'h0000, 8'h00};
   endcase

endmodule
