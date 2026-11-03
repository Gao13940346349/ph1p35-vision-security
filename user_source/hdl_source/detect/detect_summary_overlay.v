// Opaque top-left detection summary. Counts the nine frame-matched descriptors
// serially in the first ten pixels of each frame, then renders five text rows.
// Two pixel-clock output stages match hdmi_mixer and shape_overlay.
module detect_summary_overlay #(
    parameter integer PANEL_X=8,
    parameter integer PANEL_Y=8
)(
    input wire clk, rst_n, de, user, enable,
    input wire [11:0] x, y,
    input wire [422:0] boxes,
    output reg hit,
    output reg [23:0] colour
);
    reg [3:0] work_count [0:8];
    reg [3:0] shown_count [0:8];
    reg [3:0] work_total, shown_total, scan_slot;
    reg scanning;
    integer i;
    wire [4:0] category = boxes[scan_slot*47+42 +:5];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i=0; i<9; i=i+1) begin
                work_count[i]<=0;
                shown_count[i]<=0;
            end
            work_total<=0; shown_total<=0; scan_slot<=0; scanning<=0;
        end else if (user) begin
            for (i=0; i<9; i=i+1) work_count[i]<=0;
            work_total<=0; scan_slot<=0; scanning<=1;
        end else if (scanning) begin
            if (scan_slot<9) begin
                scan_slot<=scan_slot+1'b1;
                case (category)
                    5'b01_00_1: begin work_count[0]<=work_count[0]+1'b1; work_total<=work_total+1'b1; end
                    5'b01_01_1: begin work_count[1]<=work_count[1]+1'b1; work_total<=work_total+1'b1; end
                    5'b01_10_1: begin work_count[2]<=work_count[2]+1'b1; work_total<=work_total+1'b1; end
                    5'b10_00_1: begin work_count[3]<=work_count[3]+1'b1; work_total<=work_total+1'b1; end
                    5'b10_01_1: begin work_count[4]<=work_count[4]+1'b1; work_total<=work_total+1'b1; end
                    5'b10_10_1: begin work_count[5]<=work_count[5]+1'b1; work_total<=work_total+1'b1; end
                    5'b11_00_1: begin work_count[6]<=work_count[6]+1'b1; work_total<=work_total+1'b1; end
                    5'b11_01_1: begin work_count[7]<=work_count[7]+1'b1; work_total<=work_total+1'b1; end
                    5'b11_10_1: begin work_count[8]<=work_count[8]+1'b1; work_total<=work_total+1'b1; end
                    default: begin end
                endcase
            end else begin
                for (i=0; i<9; i=i+1) shown_count[i]<=work_count[i];
                shown_total<=work_total;
                scanning<=0;
            end
        end
    end

    wire [35:0] shown = {shown_count[8],shown_count[7],shown_count[6],
                         shown_count[5],shown_count[4],shown_count[3],
                         shown_count[2],shown_count[1],shown_count[0]};
    function [7:0] panel_char;
        input [2:0] row;
        input [4:0] col;
        input [3:0] total;
        input [35:0] counts;
        begin
            panel_char=" ";
            case (row)
                0: case (col)
                    0:panel_char="D"; 1:panel_char="E"; 2:panel_char="T";
                    3:panel_char="E"; 4:panel_char="C"; 5:panel_char="T";
                    6:panel_char="E"; 7:panel_char="D"; 8:panel_char=":";
                    9:panel_char="0"+total;
                endcase
                1: case (col)
                    0:panel_char="C"; 1:panel_char="O"; 2:panel_char="L";
                    3:panel_char="O"; 4:panel_char="R";
                    7:panel_char="T"; 8:panel_char="R"; 9:panel_char="I";
                    11:panel_char="S"; 12:panel_char="Q"; 13:panel_char="R";
                    15:panel_char="C"; 16:panel_char="I"; 17:panel_char="R";
                endcase
                2: case (col)
                    0:panel_char="R"; 1:panel_char="E"; 2:panel_char="D";
                    8:panel_char="0"+counts[3:0];
                    12:panel_char="0"+counts[7:4];
                    16:panel_char="0"+counts[11:8];
                endcase
                3: case (col)
                    0:panel_char="G"; 1:panel_char="R"; 2:panel_char="E";
                    3:panel_char="E"; 4:panel_char="N";
                    8:panel_char="0"+counts[15:12];
                    12:panel_char="0"+counts[19:16];
                    16:panel_char="0"+counts[23:20];
                endcase
                4: case (col)
                    0:panel_char="B"; 1:panel_char="L";
                    2:panel_char="U"; 3:panel_char="E";
                    8:panel_char="0"+counts[27:24];
                    12:panel_char="0"+counts[31:28];
                    16:panel_char="0"+counts[35:32];
                endcase
            endcase
        end
    endfunction

    wire panel = enable && de && x>=PANEL_X && x<PANEL_X+320 &&
                 y>=PANEL_Y && y<PANEL_Y+120;
    wire [11:0] text_x=x-(PANEL_X+8);
    wire [11:0] text_y=y-(PANEL_Y+8);
    wire [4:0] char_col=text_x[8:4];
    wire [3:0] char_x=text_x[3:0];
    reg [2:0] row;
    reg [3:0] font_y;
    reg row_valid;
    always @(*) begin
        row=0; font_y=0; row_valid=1;
        if (text_y<12'd16) begin row=0; font_y=text_y[3:0]; end
        else if (text_y>=12'd22 && text_y<12'd38) begin row=1; font_y=text_y-12'd22; end
        else if (text_y>=12'd44 && text_y<12'd60) begin row=2; font_y=text_y-12'd44; end
        else if (text_y>=12'd66 && text_y<12'd82) begin row=3; font_y=text_y-12'd66; end
        else if (text_y>=12'd88 && text_y<12'd104) begin row=4; font_y=text_y-12'd88; end
        else row_valid=0;
    end
    wire [7:0] ch=panel_char(row,char_col,shown_total,shown);
    wire [15:0] font_bits;
    // Use the existing two pixel clocks: row lookup first, bit select second.
    // The character decoder no longer feeds a combinational glyph/bit tree.
    osd_ascii_font_rom u_font(.I_clk(clk),.I_char(ch),.I_row(font_y),.O_row_bits(font_bits));
    reg panel_s1, text_s1;
    reg [3:0] char_x_s1;
    wire letter_s1=text_s1 && font_bits[4'd15-char_x_s1];
    reg [2:0] row_s1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin panel_s1<=0;text_s1<=0;char_x_s1<=0;row_s1<=0;hit<=0;colour<=0; end
        else begin
            panel_s1<=panel;
            text_s1<=panel && row_valid && x>=PANEL_X+8 && x<PANEL_X+296;
            char_x_s1<=char_x;
            row_s1<=row;
            hit<=panel_s1;
            if (!panel_s1) colour<=24'd0;
            else if (!letter_s1) colour<=24'h102030;
            else case (row_s1)
                2:colour<=24'hff6060;
                3:colour<=24'h60ff80;
                4:colour<=24'h80aaff;
                1:colour<=24'h00e6ff;
                default:colour<=24'hffffff;
            endcase
        end
    end
endmodule
