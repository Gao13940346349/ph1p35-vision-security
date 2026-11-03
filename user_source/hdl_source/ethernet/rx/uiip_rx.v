/*******************************MILIANKE*******************************
*Company : MiLianKe Electronic Technology Co., Ltd.
*WebSite:https://www.milianke.com
*TechWeb:https://www.uisrc.com
*tmall-shop:https://milianke.tmall.com
*jd-shop:https://milianke.jd.com
*taobao-shop1: https://milianke.taobao.com
*Create Date: 2022/12/23
*Module Name:
*File Name:
*Description: 
*The reference demo provided by Milianke is only used for learning. 
*We cannot ensure that the demo itself is free of bugs, so users 
*should be responsible for the technical problems and consequences
*caused by the use of their own products.
*Copyright: Copyright (c) MiLianKe
*All rights reserved.
*Revision: 1.1
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

/*******************************uiip_rxģ��*********************
--��������������Ƶ�uiip_rx
-��ģ����Ҫ�� 4 �����ܣ�
1.�� ui_ip_arp_rx ģ�������Ӧ�� ip ���ݰ����ڲ���ģ�� uiip_header_checksum �� IP ���ݱ���ͷ��У���މ�м��㣬�жϰ�ͷ��ȷ�ԡ�
2.���� ip ��ͷ�е�Ŀ�� ip ��ַމ�����ݰ����ˣ�����Э���ֶ��ᅨ�� udp ���� icmp ������������Э���������ݡ�
3.�� udp ��ֱ������� uiudp_rx ģ�飬�� icmp �������ڲ� uiicmp_pkg_txģ�鴦����
4.�ڲ���ģ��icmp_pkg_ctr���� uiip__arp_rxģ�������icmp����Ŀǰֻ֧�� icmp �е�ping���������
���� ping ����������ݲ�����Ӧ�� ping �������Ϣ����� ip_tx ģ�飬���� ping ������ĸ������ݴ���icmp_echo_data_fifo�С�
*********************************************************************/

`timescale 1ns / 1ps

module uiip_rx 
(
input  wire [31 :0] I_ip_local_addr,				
input  wire        	I_reset,						//����,��λ
input  wire       	I_ip_clk,						//����,ʱ��
input  wire        	I_ip_rvalid,					//����,��Ч��IP���ź�
input  wire [7 :0]  I_ip_rdata,						//����,��Ч��IP���ݰ�
output wire [31:0]  O_icmp_req_ip_addr,				//���,ICMP���İ�ԴIP��ַ(Զ��IP��ַ)
output wire       	O_icmp_req_en,					//���,ICMP����pingӦ���������ɺ�ʹ�����
output wire [15:0]  O_icmp_req_id,					//���,ICMP���İ��ı�ʶ��,��ÿһ�����͵����ݽ��б�ʶ
output wire [15:0]  O_icmp_req_sq_num,				//���,ICMP���İ������к�,��ÿһ�����ݽ��б��ı��
output wire [15:0]  O_icmp_req_checksum,			//���,ICMP���İ����ײ�У���
output wire         O_icmp_ping_echo_data_valid,	//���,ICMP���İ���echo pingӦ����Ч�ź�
output wire [7 :0] 	O_icmp_ping_echo_data,			//���,ICMP���İ���echo pingӦ����Ч����
output wire [9 :0]  O_icmp_ping_echo_data_len,		//���,ICMP���İ���echo pingӦ����Ч����
output reg        	O_udp_ip_rvalid,				//���,UDP������Ч�ź�
output reg  [7 :0]  O_udp_ip_rdata,					//���,UDP������Ч����
output wire       	O_checksum_rerror				//���,IP���ײ�У����Ƿ���ȷ
);

reg [4:0]    cnt;
reg [3:0]    ip_version;
reg [3:0]    ip_header_len;
reg [7:0]    ip_tos;
reg [15:0]   ip_pkg_len;
reg [15:0]   ip_packet_id;
reg [2:0]    ip_packet_flag;
reg [12:0]   ip_fragment_offset;
reg [7:0]    ip_packet_ttl;
reg [7:0]    ip_packet_protocol;
reg [15:0]   ip_header_checksum;
reg [31:0]   ip_src_address;
reg [31:0]   ip_dst_address;
reg          icmp_pkg_valid;
reg [1:0]    STATE;

localparam   WAIT_IP_PACKET    = 2'd0;
localparam   RECORD_IP_HEADER  = 2'd1;
localparam   OUTPUT_UDP_PACKET = 2'd2;
localparam   WAIT_PACKET_END   = 2'd3;

localparam   UDP_TYPE    = 8'h11;
localparam   ICMP_TYPE   = 8'h01;

assign  O_icmp_req_ip_addr = ip_src_address;

//ip��֡ͷУ��ͼ���
uiip_header_checksum ip_header_checksum_inst 
(
.I_reset					(I_reset), 
.I_clk						(I_ip_clk), 
.I_ip_rdata_valid			(I_ip_rvalid), 
.I_ip_rdata					(I_ip_rdata), 
.O_checksum_rerror			(O_checksum_rerror)
);

//icmp���ݱ��Ĵ���ģ��	 
uiicmp_pkg_ctrl icmp_pkg_ctrl_inst 
(
.I_reset					(I_reset), 								//��λ����
.I_clk						(I_ip_clk),								//ʱ������ 							
.I_icmp_pkg_valid			(I_ip_rvalid & icmp_pkg_valid),  		//����,��Ч��ICMP���İ��ź�
.I_icmp_pkg_data			(I_ip_rdata), 							//����,��Ч��ICMP���İ�������Ч
.O_icmp_req_en				(O_icmp_req_en), 						//���,ICMP���İ�����
.O_icmp_req_id				(O_icmp_req_id), 						//���,ICMP���İ��ı�ʶ��
.O_icmp_req_sq_num			(O_icmp_req_sq_num),					//���,ICMP���İ������к�
.O_icmp_req_checksum		(O_icmp_req_checksum),					//���,ICMP�����ײ�У���					
.O_icmp_ping_echo_data_valid(O_icmp_ping_echo_data_valid),			//���,���յ�ICMP���ģ�echo pingӦ����Ч			
.O_icmp_ping_echo_data   	(O_icmp_ping_echo_data),				//���,���յ�ICMP���ģ�echo pingӦ����Ч���ݲ���
.O_icmp_ping_echo_data_len  (O_icmp_ping_echo_data_len)				//���,���յ�ICMP���ģ�echo pingӦ����Ч���ݳ���
);

//������IP����UDP���İ�����ICMP���İ�	 
always @(posedge I_ip_clk or posedge I_reset) begin
	if(I_reset) begin
		cnt 					<= 5'd0;
		ip_version 				<= 4'd0;	//IP�ײ�-�汾:4λ���ݱ�ʾIP�汾�ţ�Ϊ4ʱ��ʾIPv4��Ϊ6ʱ��ʾIPv6��IPv4ʹ�ý϶ࡣ
		ip_header_len 			<= 4'd0;	//IP�ײ�-�ײ�����:4λ���ݱ�ʾIP�ײ�һ���ж��ٸ�32λ��4���ֽڣ����ݡ�û�п�ѡ�ֶε�IP�ײ�����Ϊ20���ֽڣ����ײ�����Ϊ5
		ip_tos 					<= 8'd0;	//IP�ײ�-��������:8λ�������ͱ����ֳ��������ֶΣ�3λ���ȼ��ֶκ�4λTOS�ֶΣ����һλ�̶�Ϊ0����������Ϊ0ʱ��ʾһ�����
		ip_pkg_len 				<= 16'd0;	//IP�ײ�-�ܳ���:16λIP���ݱ��ܳ��Ȱ���IP�ײ���IP���ݲ��֣����ֽ�Ϊ��λ������IP�ײ����Ⱥ�IP���ݱ��ܳ��ȿ��Լ����IP���ݱ����������ݵ���ʼλ�úͳ���
		ip_packet_id 			<= 16'd0;	//IP�ײ�-ID:16λ��ʶ�ֶΣ�������ʶ�������͵�ÿһ�����ݱ���ÿ����һ�ݱ�������ֵ�ͻ��1
		ip_packet_flag 			<= 3'd0;	//IP�ײ�-��־�ֶ�:3λ��־�ֶεĵ�1λ�Ǳ���λ����2λ��ʾ��ֹ��Ƭ��1��ʾ����Ƭ��0������Ƭ������3λ��ʶ�����Ƭ��ͨ��Ϊ010����Ƭ
		ip_fragment_offset 		<= 13'd0;	//IP�ײ�-Ƭƫ��:13λƬƫ�ƣ��ڽ��շ��������ݱ�����ʱ������ʶ��Ƭ��˳��
		ip_packet_ttl 			<= 8'd0;	//IP�ײ�-����ʱ��:	8λ����ʱ���ֹ��ʧ�����ݰ�������ֹ�Ĵ�����һ�㱻����Ϊ64����128
		ip_packet_protocol 		<= 8'd0;	//IP�ײ�-Э��:8λЭ�����ͱ�ʾ�����ݱ���Я���ϲ�����ʹ�õ�Э�����ͣ�ICMPΪ1��TCPΪ6��UDPΪ17
		ip_header_checksum 		<= 16'd0;	//IP�ײ�-�ײ�У���:16λ�ײ�У��ͣ����ֶ�ֻУ�����ݱ����ײ������������ݲ���
		ip_src_address 			<= 32'd0;	//IP�ײ�-ԴIP��ַ:32λ���Ͷ˵�IP��ַ
		ip_dst_address 			<= 32'd0;	//IP�ײ�-Ŀ��IP��ַ:32λ���ն˵�IP��ַ
		icmp_pkg_valid 			<= 1'b0;	//icmp���ݱ�����Ч�ź�
		O_udp_ip_rvalid 		<= 1'b0;	//UDP���ݰ���Ч
	    O_udp_ip_rdata 			<= 8'd0;	//UDP���ݰ���Ч���
		STATE 					<= WAIT_IP_PACKET;
	end
	else begin
		case(STATE)
		WAIT_IP_PACKET:begin
			if(I_ip_rvalid) begin
				ip_version 			<= I_ip_rdata[7:4];	//IP�ײ�-�汾:4λ���ݱ�ʾIP�汾��
				ip_header_len 		<= I_ip_rdata[3:0];	//IP�ײ�-�ײ�����:4λ���ݱ�ʾIP�ײ�һ���ж��ٸ�32λ��4���ֽڣ�����
				STATE 				<= RECORD_IP_HEADER; //��һ״̬����������IPͷ����Ϣ
			end
			else
				STATE 				<= WAIT_IP_PACKET;
		end
		RECORD_IP_HEADER:begin
			case(cnt)
				0: begin ip_tos 					<= I_ip_rdata; 		cnt <= cnt + 1'b1; end	//IP�ײ�-��������:8λ�������ͱ����ֳ��������ֶΣ�3λ���ȼ��ֶκ�4λTOS�ֶΣ����һλ�̶�Ϊ0����������Ϊ0ʱ��ʾһ�����
				1: begin ip_pkg_len[15:8] 			<= I_ip_rdata; 		cnt <= cnt + 1'b1; end	//IP�ײ�-�ܳ���:16λIP���ݱ��ܳ��Ȱ���IP�ײ���IP���ݲ��֣����ֽ�Ϊ��λ��
				2: begin ip_pkg_len[7:0] 			<= I_ip_rdata; 		cnt <= cnt + 1'b1; end	//IP�ײ�-�ܳ���:16λIP���ݱ��ܳ��Ȱ���IP�ײ���IP���ݲ��֣����ֽ�Ϊ��λ��
				3: begin ip_packet_id[15:8] 		<= I_ip_rdata; 		cnt <= cnt + 1'b1; end	//IP�ײ�-ID:16λ��ʶ�ֶΣ�������ʶ�������͵�ÿһ�����ݱ���
				4: begin ip_packet_id[7:0] 			<= I_ip_rdata; 		cnt <= cnt + 1'b1; end	//IP�ײ�-ID:16λ��ʶ�ֶΣ�������ʶ�������͵�ÿһ�����ݱ���
				5: begin 
					ip_packet_flag 					<= I_ip_rdata[7:5]; 						//IP�ײ�-��־�ֶ�:3λ��־�ֶεĵ�1λ�Ǳ���λ����2λ��ʾ��ֹ��Ƭ��1��ʾ����Ƭ��0������Ƭ������3λ��ʶ�����Ƭ��ͨ��Ϊ010����Ƭ
					ip_fragment_offset[12:8] 		<= I_ip_rdata[4:0];							//IP�ײ�-Ƭƫ��:13λƬƫ�ƣ��ڽ��շ��������ݱ�����ʱ������ʶ��Ƭ��˳��
					cnt 							<= cnt + 1'b1; 
				end
				6: begin ip_fragment_offset[7:0] 	<= I_ip_rdata[4:0]; cnt <= cnt + 1'b1; end 	//IP�ײ�-Ƭƫ��:13λƬƫ�ƣ��ڽ��շ��������ݱ�����ʱ������ʶ��Ƭ��˳��
				7: begin ip_packet_ttl[7:0] 		<= I_ip_rdata[4:0]; cnt <= cnt + 1'b1; end	//IP�ײ�-����ʱ��:8λ����ʱ���ֹ��ʧ�����ݰ�������ֹ�Ĵ�����һ�㱻����Ϊ64����128
				8: begin ip_packet_protocol[7:0] 	<= I_ip_rdata[4:0]; cnt <= cnt + 1'b1; end 	//IP�ײ�-Э��:8λЭ�����ͱ�ʾ�����ݱ���Я���ϲ�����ʹ�õ�Э�����ͣ�ICMPΪ1��TCPΪ6��UDPΪ17	
				9: begin ip_header_checksum[15:8] 	<= I_ip_rdata; 		cnt <= cnt + 1'b1; end	//IP�ײ�-�ײ�У���:16λ�ײ�У��ͣ����ֶ�ֻУ�����ݱ����ײ������������ݲ���
				10: begin ip_header_checksum[7:0] 	<= I_ip_rdata; 		cnt <= cnt + 1'b1; end	//IP�ײ�-�ײ�У���:16λ�ײ�У��ͣ����ֶ�ֻУ�����ݱ����ײ������������ݲ���
				11: begin ip_src_address[31:24] 	<= I_ip_rdata; 		cnt <= cnt + 1'b1; end	//IP�ײ�-ԴIP��ַ:32λ���Ͷ˵�IP��ַ
				12: begin ip_src_address[23:16] 	<= I_ip_rdata; 		cnt <= cnt + 1'b1; end	//IP�ײ�-ԴIP��ַ:32λ���Ͷ˵�IP��ַ
				13: begin ip_src_address[15:8] 		<= I_ip_rdata; 		cnt <= cnt + 1'b1; end	//IP�ײ�-ԴIP��ַ:32λ���Ͷ˵�IP��ַ
				14: begin ip_src_address[7:0] 		<= I_ip_rdata; 		cnt <= cnt + 1'b1; end	//IP�ײ�-ԴIP��ַ:32λ���Ͷ˵�IP��ַ
				15: begin ip_dst_address[31:24] 	<= I_ip_rdata; 		cnt <= cnt + 1'b1; end	//IP�ײ�-Ŀ��IP��ַ:32λ���ն˵�IP��ַ
				16: begin ip_dst_address[23:16] 	<= I_ip_rdata; 		cnt <= cnt + 1'b1; end	//IP�ײ�-Ŀ��IP��ַ:32λ���ն˵�IP��ַ
				17: begin ip_dst_address[15:8] 		<= I_ip_rdata; 		cnt <= cnt + 1'b1; end	//IP�ײ�-Ŀ��IP��ַ:32λ���ն˵�IP��ַ
				18: begin 
					ip_dst_address[7:0] 		<= I_ip_rdata; 									//IP�ײ�-Ŀ��IP��ַ:32λ���ն˵�IP��ַ
				 	cnt 						<= 5'd0; 
					if(({ip_dst_address[31:8], I_ip_rdata} == I_ip_local_addr) ||
					   ({ip_dst_address[31:8], I_ip_rdata} == 32'hFFFFFFFF)) begin		//匹配本机IP，或接受受限广播255.255.255.255
						if(ip_packet_protocol == ICMP_TYPE) begin								//�����ICMP����
							icmp_pkg_valid 		<= 1'b1;										//�����ICMP����Ч			
							STATE 				<= WAIT_PACKET_END;								//�ȴ��������״̬
						end
						else if(ip_packet_protocol == UDP_TYPE) begin							//�����UDP����Ч				
							icmp_pkg_valid 		<= 1'b0;
							STATE 				<= OUTPUT_UDP_PACKET;							//���UDP����UDPЭ���
						end
						else begin
							icmp_pkg_valid 		<= 1'b0;
							STATE 				<= WAIT_PACKET_END;								//�ȴ��������״̬
						end
					end
					else begin
						icmp_pkg_valid 			<= 1'b0;
						STATE 					<= WAIT_PACKET_END;								//�ȴ��������״̬
					end
				end
			endcase
		end
		OUTPUT_UDP_PACKET:begin
			if(I_ip_rvalid) begin 	//���ĺ�֧�������UDPЭ���
				O_udp_ip_rvalid 					<= 1'b1;
				O_udp_ip_rdata 					<= I_ip_rdata;
				STATE 							<= OUTPUT_UDP_PACKET;
			end
			else begin
				O_udp_ip_rvalid 					<= 1'b0;
				O_udp_ip_rdata 					<= 8'd0;
				STATE 							<= WAIT_IP_PACKET;
			end
        end
        WAIT_PACKET_END:begin			//�ȴ����������
			if(I_ip_rvalid)
				STATE 							<= WAIT_PACKET_END;
			else begin
				icmp_pkg_valid 					<= 1'b0;
				STATE 							<= WAIT_IP_PACKET; 
			end
		end
		endcase
	end
end


endmodule
