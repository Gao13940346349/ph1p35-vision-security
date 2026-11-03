// Frame-stable colour HE, independently implemented for this four-pixel stream.
// One rotating pixel in SAMPLE_STRIDE groups estimates a 256-bin Y histogram.
// A single histogram RAM is reused: capture a COMPLETE frame, drain pending
// read/modify/writes, build CDF-min in blanking, clear bins as they are consumed.
// If blanking is short, skip collecting the next frame; NEVER stall video or
// use a partial histogram. Four replicated, double-buffered LUT RAMs supply
// four simultaneous reads. Publish a finished table only on a frame marker.
// Serial restoring division is confined to table construction (no pixel DSP
// or divider). Bounded half-strength mapping protects noise and colour gamut.
module luma_histogram_equalizer_96 #(
    parameter integer IMG_WIDTH=1280,
    parameter integer IMG_HEIGHT=720,
    parameter integer SAMPLE_STRIDE=16
)(
    input wire clk, rst_n, enable,
    input wire [95:0] data_in,
    input wire valid_in, last_in, user_in,
    output reg [95:0] data_out,
    output wire valid_out, last_out, user_out
);
    localparam integer GROUPS=IMG_WIDTH/4;
    localparam integer TOTAL=GROUPS*IMG_HEIGHT;
    localparam integer CW=$clog2(TOTAL+1);
    localparam integer NW=CW+8;
    localparam integer SW=$clog2(SAMPLE_STRIDE);
    localparam CLEAR=0, WAIT_FRAME=1, CAPTURE=2, DRAIN=3,
                       H_READ=4, H_ACC=5, DIVIDE=6, WRITE_MAP=7;
    // synthesis translate_off
    initial begin
        if(IMG_WIDTH<8 || IMG_WIDTH%4!=0 || IMG_HEIGHT<1 || SAMPLE_STRIDE<4)
            $fatal(1,"HE requires width >=8 divisible by 4, positive height and stride >=4");
    end
    // synthesis translate_on
    reg [3:0] state;
    (* ram_style="block", syn_ramstyle="block_ram" *) reg [CW-1:0] histogram [0:255];
    (* ram_style="block", syn_ramstyle="block_ram" *) reg [7:0] map0 [0:511];
    (* ram_style="block", syn_ramstyle="block_ram" *) reg [7:0] map1 [0:511];
    (* ram_style="block", syn_ramstyle="block_ram" *) reg [7:0] map2 [0:511];
    (* ram_style="block", syn_ramstyle="block_ram" *) reg [7:0] map3 [0:511];
    reg [7:0] bin,clear_bin;
    reg [CW-1:0] sample_count,group_count;
    reg [11:0] group_x;
    reg [SW-1:0] sample_div;
    reg [1:0] lane,drain_count;
    reg [7:0] min_y,max_y;
    reg malformed,flat_frame,seen_nonzero;
    reg active_bank,build_bank,map_pending,map_valid;
    reg [CW-1:0] cdf,cdf_min,denominator;
    reg [NW-1:0] numerator,quotient;
    reg [CW:0] remainder;
    reg [5:0] divide_left;
    reg [7:0] mapped;

    function [7:0] luminance;
        input [23:0] c;
        reg [9:0] sum;
        begin sum={2'b0,c[23:16]}+{1'b0,c[15:8],1'b0}+{2'b0,c[7:0]};
              luminance=sum[9:2]; end
    endfunction
    // Limit the CDF correction to +/-64, then blend 50% with identity.
    function [7:0] regularized_map;
        input [7:0] original,equalized;
        reg signed [9:0] d,y;
        begin
            d=$signed({1'b0,equalized})-$signed({1'b0,original});
            if(d>64) d=64; else if(d< -64) d= -64;
            y=$signed({1'b0,original})+(d>>>1);
            regularized_map=y[7:0];
        end
    endfunction
    function [7:0] clip;
        input signed [9:0] v;
        begin clip=v<0 ? 8'd0 : v>255 ? 8'd255 : v[7:0]; end
    endfunction

    wire start_capture=state==WAIT_FRAME && user_in;
    wire capture_now=(state==CAPTURE && !user_in) || start_capture;
    wire [7:0] sample_y=luminance(data_in[95-24*lane -:24]);
    wire request=capture_now && valid_in && (start_capture || sample_div==0);
    reg request_d;
    reg [7:0] request_bin_d;
    reg [CW-1:0] hist_q;
    wire hist_write=state==CLEAR || state==WRITE_MAP || request_d;
    wire [7:0] hist_write_addr=state==CLEAR ? clear_bin :
                                    state==WRITE_MAP ? bin : request_bin_d;
    wire [CW-1:0] hist_write_data=(state==CLEAR || state==WRITE_MAP) ?
                                    {CW{1'b0}} : hist_q+1'b1;
    wire hist_read=request || state==H_READ;
    wire [7:0] hist_read_addr=request ? sample_y : bin;
    // Spaced sampling leaves a full read + increment + write before a bin can
    // recur. No same-bin write collision, no four-way histogram replication.
    always @(posedge clk) begin
        if(hist_write) histogram[hist_write_addr]<=hist_write_data;
        if(hist_read) hist_q<=histogram[hist_read_addr];
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin request_d<=0;request_bin_d<=0;end
        else begin request_d<=request;request_bin_d<=sample_y;end
    end
    wire [CW-1:0] cdf_next=cdf+hist_q;
    wire [CW-1:0] cdf_offset=cdf_next-cdf_min;
    wire [NW-1:0] expanded_offset={{8{1'b0}},cdf_offset};
    wire [CW+1:0] trial={remainder,numerator[NW-1]};
    wire [CW+1:0] divisor={2'b0,denominator};
    wire divide_bit=trial>=divisor;
    wire [CW+1:0] next_rem=divide_bit ? trial-divisor : trial;
    wire [7:0] table_value=flat_frame ? bin : regularized_map(bin,mapped);
    always @(posedge clk) begin
        if(state==WRITE_MAP) begin
            map0[{build_bank,bin}]<=table_value;
            map1[{build_bank,bin}]<=table_value;
            map2[{build_bank,bin}]<=table_value;
            map3[{build_bank,bin}]<=table_value;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=CLEAR;clear_bin<=0;bin<=0;sample_count<=0;group_count<=0;group_x<=0;
            sample_div<=0;lane<=0;drain_count<=0;min_y<=255;max_y<=0;
            malformed<=0;flat_frame<=0;seen_nonzero<=0;
            active_bank<=0;build_bank<=1;map_pending<=0;map_valid<=0;
            cdf<=0;cdf_min<=0;denominator<=0;numerator<=0;quotient<=0;
            remainder<=0;divide_left<=0;mapped<=0;
        end else begin
            if(user_in && map_pending) begin
                active_bank<=build_bank;map_valid<=1;map_pending<=0;
            end
            case(state)
                CLEAR:begin
                    clear_bin<=clear_bin+1'b1;
                    if(clear_bin==255) state<=WAIT_FRAME;
                end
                WAIT_FRAME:if(user_in) begin
                    state<=CAPTURE;group_count<=0;group_x<=0;sample_count<=0;
                    sample_div<=0;min_y<=255;max_y<=0;malformed<=0;
                end
                CAPTURE:if(user_in) begin
                    // A new SOF before expected frame end invalidates the old
                    // statistics. Clear them and wait for another COMPLETE SOF.
                    malformed<=1;drain_count<=0;state<=DRAIN;
                end
                DRAIN:begin
                    drain_count<=drain_count+1'b1;
                    if(drain_count==3) begin
                        if(malformed || sample_count==0) begin state<=CLEAR;clear_bin<=0;end
                        else begin
                            state<=H_READ;bin<=0;cdf<=0;cdf_min<=0;seen_nonzero<=0;
                            flat_frame<=min_y==max_y;build_bank<=~active_bank;
                        end
                    end
                end
                H_READ:state<=H_ACC;
                H_ACC:begin
                    cdf<=cdf_next;
                    mapped<=0;
                    if(flat_frame || !seen_nonzero) begin
                        if(hist_q!=0) begin
                            seen_nonzero<=1;cdf_min<=cdf_next;denominator<=sample_count-cdf_next;
                        end
                        state<=WRITE_MAP;
                    end else begin
                        // (CDF-CDFmin)*255, explicitly widened before shifts.
                        numerator<=(expanded_offset<<8)-expanded_offset;
                        quotient<=0;remainder<=0;divide_left<=NW;state<=DIVIDE;
                    end
                end
                DIVIDE:begin
                    numerator<=numerator<<1;
                    remainder<=next_rem[CW:0];
                    quotient<={quotient[NW-2:0],divide_bit};
                    divide_left<=divide_left-1'b1;
                    if(divide_left==1) begin
                        mapped<={quotient[6:0],divide_bit};state<=WRITE_MAP;
                    end
                end
                WRITE_MAP:begin
                    if(bin==255) begin map_pending<=1;state<=WAIT_FRAME;end
                    else begin bin<=bin+1'b1;state<=H_READ;end
                end
                default:begin state<=CLEAR;clear_bin<=0;end
            endcase
            if(capture_now && valid_in) begin
                group_count<=start_capture ? 1 : group_count+1'b1;
                group_x<=last_in ? 0 : (start_capture ? 1 : group_x+1'b1);
                if(last_in != ((start_capture ? 0 : group_x)==GROUPS-1)) malformed<=1;
                if(start_capture) sample_div<=1;
                else if(sample_div==SAMPLE_STRIDE-1) sample_div<=0;
                else sample_div<=sample_div+1'b1;
                if(request) begin
                    sample_count<=start_capture ? 1 : sample_count+1'b1;
                    min_y<=start_capture || sample_y<min_y ? sample_y : min_y;
                    max_y<=start_capture || sample_y>max_y ? sample_y : max_y;
                    lane<=lane+1'b1;
                end
                if((start_capture ? 0 : group_count)==TOTAL-1) begin
                    state<=DRAIN;drain_count<=0;
                end
            end
        end
    end

    // Five pixel stages: RGB/Y sum, Y, synchronous LUT, RGB correction, clip.
    reg [2:0] control [0:4];
    reg [95:0] rgb1,rgb2,rgb3;
    reg [9:0] ysum1 [0:3];
    reg [7:0] y2 [0:3],y3 [0:3],lookup [0:3];
    reg use_map1,use_map2,use_map3,bank1,bank2;
    reg signed [9:0] corrected [0:11];
    integer p,c;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            for(p=0;p<5;p=p+1) control[p]<=0;
            use_map1<=0;use_map2<=0;use_map3<=0;bank1<=0;bank2<=0;
        end else begin
            control[0]<={user_in,last_in,valid_in};
            for(p=1;p<5;p=p+1) control[p]<=control[p-1];
            use_map1<=enable && (map_valid || (user_in && map_pending));
            use_map2<=use_map1;
            use_map3<=use_map2;
            bank1<=(user_in && map_pending) ? build_bank : active_bank;
            bank2<=bank1;
        end
    end
    assign {user_out,last_out,valid_out}=control[4];
    always @(posedge clk) begin
        rgb1<=data_in;rgb2<=rgb1;rgb3<=rgb2;
        for(c=0;c<4;c=c+1) begin
            ysum1[c]<={2'b0,data_in[95-24*c -:8]}+
                      {1'b0,data_in[87-24*c -:8],1'b0}+{2'b0,data_in[79-24*c -:8]};
            y2[c]<=ysum1[c][9:2];y3[c]<=y2[c];
        end
        lookup[0]<=map0[{bank2,y2[0]}];lookup[1]<=map1[{bank2,y2[1]}];
        lookup[2]<=map2[{bank2,y2[2]}];lookup[3]<=map3[{bank2,y2[3]}];
        for(c=0;c<12;c=c+1) begin
            corrected[c]<=$signed({1'b0,rgb3[95-8*c -:8]})+
                (use_map3 ? $signed({1'b0,lookup[c/3]})-$signed({1'b0,y3[c/3]}) : 10'sd0);
            data_out[95-8*c -:8]<=clip(corrected[c]);
        end
    end
endmodule
