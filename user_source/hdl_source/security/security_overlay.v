// Standalone security application OSD. Two registers match hdmi_mixer DE_2d.
// Business state is held for one display frame; no image buffer or arithmetic
// divider is added. Existing shape/tracking overlays provide target annotation.
module security_overlay #(
    parameter integer SCREEN_WIDTH=1280,
    parameter integer SCREEN_HEIGHT=720,
    parameter integer PANEL_X=176,
    parameter integer PANEL_Y=8
)(
    input wire clk,rst_n,de,user,enable,
    input wire [11:0] x,y,
    input wire [2:0] app_state,severity,
    input wire [15:0] high_count_bcd,suspicious_count_bcd,event_source_id,
    output reg hit,
    output reg [23:0] colour
);
    localparam integer SAFE_END=SCREEN_WIDTH*2/5;
    localparam integer RESTRICT_START=SCREEN_WIDTH*3/5;
    localparam [23:0] BG=24'h102030,WHITE=24'heaf2ff;
    localparam [23:0] GREEN=24'h50e890,BLUE=24'h70b8ff,RED=24'hff6060;
    localparam [23:0] YELLOW=24'hffe070,INACTIVE=24'h738498;
    reg [2:0] frame_state,frame_severity;
    reg [15:0] frame_high,frame_suspicious,frame_source;
    reg [5:0] blink_frame;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            frame_state<=0;frame_severity<=0;frame_high<=0;
            frame_suspicious<=0;frame_source<=0;blink_frame<=0;
        end else if(user) begin
            frame_state<=app_state;frame_severity<=severity;
            frame_high<=high_count_bcd;frame_suspicious<=suspicious_count_bcd;
            frame_source<=event_source_id;blink_frame<=blink_frame+1'b1;
        end
    end
    function [7:0] hex_char;
        input [3:0] n;
        begin hex_char=n<10 ? "0"+n : "A"+(n-10);end
    endfunction
    function [7:0] status_char;
        input [4:0] col;
        begin
            status_char=" ";
            case(frame_state)
                0:case(col)
                    0: status_char="H";
                    1: status_char="O";
                    2: status_char="M";
                    3: status_char="E";
                    5: status_char="/";
                    7: status_char="S";
                    8: status_char="E";
                    9: status_char="T";
                    10: status_char="T";
                    11: status_char="L";
                    12: status_char="E";
                endcase
                1:begin
                    if(frame_severity!=0) case(col)
                    0: status_char="W";
                    1: status_char="A";
                    2: status_char="R";
                    3: status_char="N";
                    4: status_char="I";
                    5: status_char="N";
                    6: status_char="G";
                    endcase
                    else case(col)
                    0: status_char="M";
                    1: status_char="O";
                    2: status_char="N";
                    3: status_char="I";
                    4: status_char="T";
                    5: status_char="O";
                    6: status_char="R";
                    endcase
                end
                2:begin
                    if(frame_severity==4) case(col)
                    0: status_char="H";
                    1: status_char="I";
                    2: status_char="G";
                    3: status_char="H";
                    5: status_char="A";
                    6: status_char="L";
                    7: status_char="A";
                    8: status_char="R";
                    9: status_char="M";
                    endcase
                    else case(col)
                    0: status_char="S";
                    1: status_char="U";
                    2: status_char="S";
                    3: status_char="P";
                    4: status_char="E";
                    5: status_char="C";
                    6: status_char="T";
                    8: status_char="E";
                    9: status_char="V";
                    10: status_char="E";
                    11: status_char="N";
                    12: status_char="T";
                    endcase
                end
                default:case(col)
                    0: status_char="A";
                    1: status_char="C";
                    2: status_char="K";
                    4: status_char="/";
                    6: status_char="W";
                    7: status_char="A";
                    8: status_char="I";
                    9: status_char="T";
                    11: status_char="C";
                    12: status_char="L";
                    13: status_char="E";
                    14: status_char="A";
                    15: status_char="R";
                endcase
            endcase
        end
    endfunction
    function [7:0] panel_char;
        input [1:0] row;
        input [5:0] col;
        begin
            panel_char=" ";
            case(row)
                0:begin
                    case(col)
                    0: panel_char="S";
                    1: panel_char="E";
                    2: panel_char="C";
                    3: panel_char="U";
                    4: panel_char="R";
                    5: panel_char="I";
                    6: panel_char="T";
                    7: panel_char="Y";
                    32: panel_char="H";
                    33: panel_char="I";
                    34: panel_char="G";
                    35: panel_char="H";
                    36: panel_char=":";
                    44: panel_char="S";
                    45: panel_char="U";
                    46: panel_char="S";
                    47: panel_char="P";
                    48: panel_char=":";
                        37:panel_char="0"+frame_high[15:12];
                        38:panel_char="0"+frame_high[11:8];
                        39:panel_char="0"+frame_high[7:4];
                        40:panel_char="0"+frame_high[3:0];
                        49:panel_char="0"+frame_suspicious[15:12];
                        50:panel_char="0"+frame_suspicious[11:8];
                        51:panel_char="0"+frame_suspicious[7:4];
                        52:panel_char="0"+frame_suspicious[3:0];
                        default:if(col>=10 && col<26) panel_char=status_char(col-10);
                    endcase
                end
                1:begin
                    if(col<13) begin
                        if(frame_state==2) case(col)
                    0: panel_char="E";
                    1: panel_char="V";
                    2: panel_char="E";
                    3: panel_char="N";
                    4: panel_char="T";
                    6: panel_char="T";
                    7: panel_char="R";
                    8: panel_char="A";
                    9: panel_char="C";
                    10: panel_char="K";
                        endcase
                        else case(col)
                    0: panel_char="C";
                    1: panel_char="A";
                    2: panel_char="M";
                    4: panel_char="F";
                    5: panel_char="I";
                    6: panel_char="X";
                    7: panel_char="E";
                    8: panel_char="D";
                        endcase
                    end else if(col>=16 && col<35) begin
                        if(frame_state==2) case(col)
                    16: panel_char="M";
                    17: panel_char="O";
                    18: panel_char="N";
                    19: panel_char="I";
                    20: panel_char="T";
                    21: panel_char="O";
                    22: panel_char="R";
                    24: panel_char="P";
                    25: panel_char="A";
                    26: panel_char="U";
                    27: panel_char="S";
                    28: panel_char="E";
                    29: panel_char="D";
                        endcase
                        else if(frame_state==1) case(col)
                    16: panel_char="Z";
                    17: panel_char="O";
                    18: panel_char="N";
                    19: panel_char="E";
                    21: panel_char="C";
                    22: panel_char="H";
                    23: panel_char="E";
                    24: panel_char="C";
                    25: panel_char="K";
                    27: panel_char="A";
                    28: panel_char="C";
                    29: panel_char="T";
                    30: panel_char="I";
                    31: panel_char="V";
                    32: panel_char="E";
                        endcase
                        else if(frame_state==0) case(col)
                    16: panel_char="W";
                    17: panel_char="A";
                    18: panel_char="I";
                    19: panel_char="T";
                    21: panel_char="S";
                    22: panel_char="T";
                    23: panel_char="A";
                    24: panel_char="B";
                    25: panel_char="L";
                    26: panel_char="E";
                        endcase
                        else case(col)
                    16: panel_char="R";
                    17: panel_char="E";
                    18: panel_char="M";
                    19: panel_char="O";
                    20: panel_char="V";
                    21: panel_char="E";
                    23: panel_char="T";
                    24: panel_char="H";
                    25: panel_char="R";
                    26: panel_char="E";
                    27: panel_char="A";
                    28: panel_char="T";
                        endcase
                    end else case(col)
                    40: panel_char="F";
                    41: panel_char="R";
                    42: panel_char="A";
                    43: panel_char="M";
                    44: panel_char="E";
                    45: panel_char=":";
                        46:panel_char=hex_char(frame_source[15:12]);
                        47:panel_char=hex_char(frame_source[11:8]);
                        48:panel_char=hex_char(frame_source[7:4]);
                        49:panel_char=hex_char(frame_source[3:0]);
                        50:panel_char="H";
                    endcase
                end
                2:case(col)
                    0: panel_char="C";
                    1: panel_char="I";
                    2: panel_char="R";
                    4: panel_char="P";
                    5: panel_char="E";
                    6: panel_char="R";
                    7: panel_char="S";
                    8: panel_char="O";
                    9: panel_char="N";
                    12: panel_char="S";
                    13: panel_char="Q";
                    14: panel_char="R";
                    16: panel_char="E";
                    17: panel_char="Q";
                    18: panel_char="U";
                    19: panel_char="I";
                    20: panel_char="P";
                    23: panel_char="T";
                    24: panel_char="R";
                    25: panel_char="I";
                    27: panel_char="V";
                    28: panel_char="E";
                    29: panel_char="H";
                    30: panel_char="I";
                    31: panel_char="C";
                    32: panel_char="L";
                    33: panel_char="E";
                    37: panel_char="K";
                    38: panel_char="8";
                    40: panel_char="A";
                    41: panel_char="C";
                    42: panel_char="K";
                endcase
                3:case(col)
                    0: panel_char="G";
                    1: panel_char="R";
                    2: panel_char="E";
                    3: panel_char="E";
                    4: panel_char="N";
                    6: panel_char="A";
                    7: panel_char="U";
                    8: panel_char="T";
                    9: panel_char="H";
                    12: panel_char="B";
                    13: panel_char="L";
                    14: panel_char="U";
                    15: panel_char="E";
                    17: panel_char="V";
                    18: panel_char="I";
                    19: panel_char="S";
                    20: panel_char="I";
                    21: panel_char="T";
                    22: panel_char="O";
                    23: panel_char="R";
                    26: panel_char="R";
                    27: panel_char="E";
                    28: panel_char="D";
                    30: panel_char="R";
                    31: panel_char="E";
                    32: panel_char="S";
                    33: panel_char="T";
                    34: panel_char="R";
                    35: panel_char="I";
                    36: panel_char="C";
                    37: panel_char="T";
                endcase
            endcase
        end
    endfunction
    function [7:0] zone_char;
        input [1:0] zone;
        input [3:0] col;
        begin
            zone_char=" ";
            case(zone)
                0:case(col)
                    0: zone_char="S";
                    1: zone_char="A";
                    2: zone_char="F";
                    3: zone_char="E";
                endcase
                1:case(col)
                    0: zone_char="W";
                    1: zone_char="A";
                    2: zone_char="R";
                    3: zone_char="N";
                    4: zone_char="I";
                    5: zone_char="N";
                    6: zone_char="G";
                endcase
                default:case(col)
                    0: zone_char="R";
                    1: zone_char="E";
                    2: zone_char="S";
                    3: zone_char="T";
                    4: zone_char="R";
                    5: zone_char="I";
                    6: zone_char="C";
                    7: zone_char="T";
                    8: zone_char="E";
                    9: zone_char="D";
                endcase
            endcase
        end
    endfunction
    wire inside=enable && de && x<SCREEN_WIDTH && y<SCREEN_HEIGHT;
    wire panel=inside && x>=PANEL_X && x<PANEL_X+896 && y>=PANEL_Y && y<PANEL_Y+96;
    wire [11:0] text_x=x-(PANEL_X+8),text_y=y-(PANEL_Y+8);
    reg [1:0] text_row;
    reg [3:0] font_y;
    reg text_valid;
    always @* begin
        text_row=0;font_y=0;text_valid=1;
        if(text_y<16) begin text_row=0;font_y=text_y[3:0];end
        else if(text_y>=22 && text_y<38) begin text_row=1;font_y=text_y-22;end
        else if(text_y>=44 && text_y<60) begin text_row=2;font_y=text_y-44;end
        else if(text_y>=66 && text_y<82) begin text_row=3;font_y=text_y-66;end
        else text_valid=0;
    end
    // Three small labels centered over each zone; labels and panel share one
    // font ROM because their screen areas do not overlap.
    wire [1:0] zone=x<SAFE_END ? 2'd0 : x<RESTRICT_START ? 2'd1 : 2'd2;
    wire [11:0] label_left=zone==0 ? SAFE_END/2-32 :
        zone==1 ? (SAFE_END+RESTRICT_START)/2-56 : (RESTRICT_START+SCREEN_WIDTH)/2-80;
    wire [11:0] label_width=zone==0 ? 64 : zone==1 ? 112 : 160;
    wire label=inside && y>=112 && y<136 && x>=label_left-4 && x<label_left+label_width+4;
    wire [11:0] label_x=x-label_left;
    wire label_text=label && y>=116 && y<132 && x>=label_left && x<label_left+label_width;
    wire [7:0] character=label ? zone_char(zone,label_x[7:4]) : panel_char(text_row,text_x[9:4]);
    wire [3:0] character_y=label ? y-116 : font_y;
    wire [15:0] font_bits;
    // Clock 1 selects the row in ERAM; clock 2 selects its pixel. Keeping
    // character selection out of the glyph/bit mux removes the long x path.
    osd_ascii_font_rom u_font(.I_clk(clk),.I_char(character),
        .I_row(character_y),.O_row_bits(font_bits));
    wire text_pixel=label_text ||
        (panel && text_valid && x>=PANEL_X+8 && x<PANEL_X+888);
    wire zone_edge=inside && y>=144 && y<SCREEN_HEIGHT-1 &&
        (((x==1 || x==SAFE_END-1 || x==SAFE_END ||
           x==RESTRICT_START-1 || x==RESTRICT_START || x==SCREEN_WIDTH-2) && !y[3]) ||
         ((y==144 || y==SCREEN_HEIGHT-2) && !x[3]));
    wire [23:0] zone_colour=frame_state==2 ? INACTIVE : zone==0 ? GREEN : zone==1 ? BLUE : RED;
    wire [23:0] state_colour=frame_state==3 ? BLUE :
        frame_state==2 ? (frame_severity==4 ? RED : YELLOW) :
        frame_severity==2 ? RED : frame_severity!=0 ? YELLOW : GREEN;
    wire stripe=panel && x<PANEL_X+4 && (frame_state!=2 || !blink_frame[4]);
    reg panel_s1,label_s1,text_pixel_s1,edge_s1,stripe_s1;
    reg [3:0] font_col_s1;
    wire letter_s1=text_pixel_s1 && font_bits[4'd15-font_col_s1];
    reg [23:0] text_colour_s1,zone_colour_s1,state_colour_s1;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            panel_s1<=0;label_s1<=0;text_pixel_s1<=0;edge_s1<=0;stripe_s1<=0;
            font_col_s1<=0;
            text_colour_s1<=0;zone_colour_s1<=0;state_colour_s1<=0;
            hit<=0;colour<=0;
        end else begin
            panel_s1<=panel;label_s1<=label;text_pixel_s1<=text_pixel;
            font_col_s1<=label ? label_x[3:0] : text_x[3:0];
            edge_s1<=zone_edge;stripe_s1<=stripe;
            zone_colour_s1<=zone_colour;state_colour_s1<=state_colour;
            text_colour_s1<=label ? zone_colour :
                text_row==0 && text_x>=160 && text_x<416 ? state_colour :
                text_row==1 && frame_state==2 ? YELLOW : WHITE;
            hit<=panel_s1 || label_s1 || edge_s1;
            if(stripe_s1) colour<=state_colour_s1;
            else if(panel_s1 || label_s1) colour<=letter_s1 ? text_colour_s1 : BG;
            else if(edge_s1) colour<=zone_colour_s1;
            else colour<=0;
        end
    end
endmodule
