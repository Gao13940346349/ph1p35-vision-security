
module image_correction #(
    parameter DATA_WIDTH   = 40,
    parameter IMG_WIDTH = 1024,
    parameter IMG_HEIGHT = 600
)(
    input                          	  I_clk   			,
    input                          	  I_rst_n 			,

    input    [DATA_WIDTH - 1:0]       I_raw_data		,       
    input                             I_raw_valid 		,     
    input                             I_raw_frame_start	,
    input                             I_raw_frame_end  	,
    // RAW10 stream output, four pixels per beat.
    output     [DATA_WIDTH - 1:0]     O_raw_tdata 		,
    output                            O_raw_tlast 		,
    output                            O_raw_tvalid		,
    output                            O_raw_tuser 		,
    input                             O_raw_tready
);

    reg                        I_raw_tvalid_d;
    reg                        I_raw_tlast_d;
    reg                        I_raw_tuser_d;
    reg    [DATA_WIDTH - 1:0]  I_raw_tdata_r;
    reg    [14:0]              h_cnt,v_cnt;
    wire                       V_valid,H_valid;
    wire                       tuser;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n )begin
            I_raw_tvalid_d <= 0;
            I_raw_tlast_d <= 0;
            I_raw_tuser_d <= 0;
            I_raw_tdata_r  <= 0;
        end
        else begin
            I_raw_tvalid_d <= I_raw_valid;
            I_raw_tlast_d <= I_raw_valid && I_raw_frame_end;
            I_raw_tuser_d <= I_raw_valid && I_raw_frame_start;
            I_raw_tdata_r  <= I_raw_data;
        end
    end
    
    // Use the real line marker from uial2axis, not elapsed blank cycles.
    // The old 8-bit blank counter wrapped and invented extra rows at long gaps.
    wire last = I_raw_tvalid_d && I_raw_tlast_d;
    // Count actual completed lines.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n || I_raw_frame_start)
            v_cnt <= 0;
        else if(last)
            v_cnt  <=  v_cnt + 1;
    end

    assign V_valid = (v_cnt < IMG_HEIGHT);

    // Count four-pixel groups in each line.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n || I_raw_frame_start) 
            h_cnt <= 0;
        else if(last)
            h_cnt <= 0;
        else if(I_raw_tvalid_d)
            h_cnt <=  h_cnt + 1;
    end
    assign H_valid = (h_cnt < IMG_WIDTH/4);
    
    assign tuser = I_raw_tuser_d;
    
    assign O_raw_tdata  = V_valid && H_valid? I_raw_tdata_r:0;
    assign O_raw_tvalid = H_valid && V_valid && I_raw_tvalid_d;
    assign O_raw_tlast  = V_valid && H_valid && last;
    assign O_raw_tuser  = tuser;

//   cwc cwc_inst
// (
// .probe0  (I_raw_data			),
// .probe1  (I_raw_valid 		),
// .probe2  (I_raw_frame_start	),
// .probe3  (I_raw_frame_end  	),
// .probe4  (O_raw_tdata 		),
// .probe5  (O_raw_tlast 		),
// .probe6  (O_raw_tvalid		),
// .probe7  (O_raw_tuser 		),
// .probe8  (h_cnt 				),
// .probe9  (v_cnt 				),
// .probe10 (cnt 				),
// .clk	  (I_clk				)
// );


endmodule
