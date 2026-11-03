// Single-target annotation, held for the displayed source frame. The selected
// descriptor and identity must be matched to DDR video before reaching here.
// The two output stages run in parallel with hdmi_mixer's existing DE_2d.
module tracking_overlay #(
    parameter integer SCREEN_WIDTH=1280,
    parameter integer SCREEN_HEIGHT=720,
    parameter integer H_OFFSET=0,
    parameter integer V_OFFSET=0,
    parameter integer PANEL_X=512,
    parameter integer PANEL_Y=8,
    parameter integer SECURITY_SUPPORT=0
)(
    input wire clk,rst_n,de,user,enable,
    input wire hide_panel,
    input wire [11:0] x,y,
    input wire [46:0] tracked_box,
    input wire [7:0] track_id,
    input wire [1:0] track_state,
    output reg hit,
    output reg [23:0] colour
);
    localparam HOLD=2'd0,ACQUIRE=2'd1,LOCKED=2'd2,LOST=2'd3;
    localparam [23:0] PANEL_BG=24'h102030, TEXT=24'hffffff;
    localparam [23:0] LOCK_COLOUR=24'hfff200,CENTRE_COLOUR=24'h00e6ff;
    reg [46:0] frame_box;
    reg [7:0] frame_id;
    reg [1:0] frame_state;
    reg [11:0] centre_x,centre_y;
    reg [10:0] bx,by,id_binary;
    reg [15:0] decimal_x,decimal_y,shown_x,shown_y;
    reg [11:0] decimal_id,shown_id;
    reg [3:0] decimal_step;
    reg converting;
    reg [15:0] adjusted_x,adjusted_y;
    reg [11:0] adjusted_id;
    integer d;
    wire [11:0] sum_x={1'b0,tracked_box[41:31]}+{1'b0,tracked_box[30:20]};
    wire [10:0] sum_y={1'b0,tracked_box[19:10]}+{1'b0,tracked_box[9:0]};
    wire [11:0] new_centre_x={1'b0,sum_x[11:1]}+H_OFFSET;
    wire [11:0] new_centre_y={2'd0,sum_y[10:1]}+V_OFFSET;

    // Convert once at the beginning of a display frame; the panel starts at
    // y=8, leaving ample time for the eleven shift/add cycles. No divider or
    // multiplier is present in the pixel path.
    always @(*) begin
        adjusted_x=decimal_x; adjusted_y=decimal_y; adjusted_id=decimal_id;
        for(d=0;d<4;d=d+1) begin
            if(decimal_x[d*4 +:4]>=5) adjusted_x[d*4 +:4]=decimal_x[d*4 +:4]+4'd3;
            if(decimal_y[d*4 +:4]>=5) adjusted_y[d*4 +:4]=decimal_y[d*4 +:4]+4'd3;
        end
        for(d=0;d<3;d=d+1)
            if(decimal_id[d*4 +:4]>=5) adjusted_id[d*4 +:4]=decimal_id[d*4 +:4]+4'd3;
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            frame_box<=0;frame_id<=0;frame_state<=HOLD;
            centre_x<=0;centre_y<=0;bx<=0;by<=0;id_binary<=0;
            decimal_x<=0;decimal_y<=0;decimal_id<=0;
            shown_x<=0;shown_y<=0;shown_id<=0;decimal_step<=0;converting<=0;
        end else if(user) begin
            frame_box<=tracked_box;frame_id<=track_id;frame_state<=track_state;
            centre_x<=new_centre_x;centre_y<=new_centre_y;
            bx<=new_centre_x[10:0];by<=new_centre_y[10:0];id_binary<={3'd0,track_id};
            decimal_x<=0;decimal_y<=0;decimal_id<=0;decimal_step<=0;converting<=1;
        end else if(converting) begin
            if(decimal_step<11) begin
                decimal_x<={adjusted_x[14:0],bx[10]};
                decimal_y<={adjusted_y[14:0],by[10]};
                decimal_id<={adjusted_id[10:0],id_binary[10]};
                bx<={bx[9:0],1'b0};by<={by[9:0],1'b0};id_binary<={id_binary[9:0],1'b0};
                decimal_step<=decimal_step+1'b1;
            end else begin
                shown_x<=decimal_x;shown_y<=decimal_y;shown_id<=decimal_id;converting<=0;
            end
        end
    end

    wire valid=frame_box[42] && frame_box[46:45]!=0 && frame_box[44:43]<3 &&
               frame_box[41:31]<=frame_box[30:20] && frame_box[19:10]<=frame_box[9:0];
    wire [11:0] x0={1'b0,frame_box[41:31]}+H_OFFSET;
    wire [11:0] x1={1'b0,frame_box[30:20]}+H_OFFSET;
    wire [11:0] y0={2'd0,frame_box[19:10]}+V_OFFSET;
    wire [11:0] y1={2'd0,frame_box[9:0]}+V_OFFSET;
    wire [11:0] centre_dx=x>=centre_x ? x-centre_x : centre_x-x;
    wire [11:0] centre_dy=y>=centre_y ? y-centre_y : centre_y-y;
    wire [11:0] screen_dx=x>=H_OFFSET+SCREEN_WIDTH/2 ?
                        x-(H_OFFSET+SCREEN_WIDTH/2) : (H_OFFSET+SCREEN_WIDTH/2)-x;
    wire [11:0] screen_dy=y>=V_OFFSET+SCREEN_HEIGHT/2 ?
                        y-(V_OFFSET+SCREEN_HEIGHT/2) : (V_OFFSET+SCREEN_HEIGHT/2)-y;
    wire inside=enable && de && x>=H_OFFSET && y>=V_OFFSET &&
                x<SCREEN_WIDTH+H_OFFSET && y<SCREEN_HEIGHT+V_OFFSET;
    wire selected=inside && valid && frame_state==LOCKED &&
        ((x>=x0 && x<=x1 && y>=y0 && y<=y1 &&
          (x-x0<4 || x1-x<4 || y-y0<4 || y1-y<4)) ||
         (centre_dx<=9 && centre_dy<=1) || (centre_dx<=1 && centre_dy<=9));
    wire centre_mark=inside && ((screen_dx<=7 && screen_dy==0) ||
                               (screen_dx==0 && screen_dy<=7));

    function [7:0] state_char;
        input [1:0] state;
        input [3:0] col;
        begin
            state_char=" ";
            case(state)
                HOLD:case(col) 0:state_char="H";1:state_char="O";2:state_char="L";3:state_char="D";endcase
                ACQUIRE:case(col) 0:state_char="A";1:state_char="C";2:state_char="Q";3:state_char="U";4:state_char="I";5:state_char="R";6:state_char="E";endcase
                LOCKED:case(col) 0:state_char="L";1:state_char="O";2:state_char="C";3:state_char="K";4:state_char="E";5:state_char="D";endcase
                LOST:case(col) 0:state_char="L";1:state_char="O";2:state_char="S";3:state_char="T";endcase
            endcase
        end
    endfunction
    function [7:0] colour_char;
        input [1:0] c;
        input [4:0] col;
        begin
            colour_char=" ";
            case(c)
                1:case(col) 0:colour_char="R";1:colour_char="E";2:colour_char="D";endcase
                2:case(col) 0:colour_char="G";1:colour_char="R";2:colour_char="E";3:colour_char="E";4:colour_char="N";endcase
                3:case(col) 0:colour_char="B";1:colour_char="L";2:colour_char="U";3:colour_char="E";endcase
            endcase
        end
    endfunction
    function [7:0] shape_char;
        input [1:0] s;
        input [4:0] col;
        begin
            shape_char=" ";
            case(s)
                0:case(col) 0:shape_char="T";1:shape_char="R";2:shape_char="I";3:shape_char="A";4:shape_char="N";5:shape_char="G";6:shape_char="L";7:shape_char="E";endcase
                1:case(col) 0:shape_char="S";1:shape_char="Q";2:shape_char="U";3:shape_char="A";4:shape_char="R";5:shape_char="E";endcase
                2:case(col) 0:shape_char="C";1:shape_char="I";2:shape_char="R";3:shape_char="C";4:shape_char="L";5:shape_char="E";endcase
            endcase
        end
    endfunction
    function [7:0] panel_char;
        input [1:0] row;
        input [4:0] col;
        begin
            panel_char=" ";
            case(row)
                0:begin
                    case(col)
                        0:panel_char="T";1:panel_char="R";2:panel_char="A";3:panel_char="C";4:panel_char="K";
                        6:panel_char="I";7:panel_char="D";8:panel_char=":";
                        9:panel_char="0"+shown_id[11:8];10:panel_char="0"+shown_id[7:4];11:panel_char="0"+shown_id[3:0];
                        default:if(col>=13 && col<20) panel_char=state_char(frame_state,col-13);
                    endcase
                end
                1:begin
                    if(valid) panel_char=col<7 ? colour_char(frame_box[46:45],col) : shape_char(frame_box[44:43],col-7);
                    else case(col) 0:panel_char="N";1:panel_char="O";2:panel_char="N";3:panel_char="E";endcase
                end
                2:begin
                    case(col)
                        0:panel_char="X";1:panel_char=":";7:panel_char="Y";8:panel_char=":";
                        2:if(valid) panel_char="0"+shown_x[15:12];
                        3:if(valid) panel_char="0"+shown_x[11:8];
                        4:if(valid) panel_char="0"+shown_x[7:4];
                        5:if(valid) panel_char="0"+shown_x[3:0];
                        9:if(valid) panel_char="0"+shown_y[15:12];
                        10:if(valid) panel_char="0"+shown_y[11:8];
                        11:if(valid) panel_char="0"+shown_y[7:4];
                        12:if(valid) panel_char="0"+shown_y[3:0];
                    endcase
                end
                3:case(col)
                    0:panel_char="K";1:panel_char="5";2:panel_char=" ";3:panel_char=" ";4:panel_char=":";
                    5:panel_char="N";6:panel_char="E";7:panel_char="X";8:panel_char="T";
                    10:panel_char="T";11:panel_char="A";12:panel_char="R";13:panel_char="G";14:panel_char="E";15:panel_char="T";
                endcase
            endcase
        end
    endfunction
    wire panel=enable && (!SECURITY_SUPPORT || !hide_panel) && de && x>=PANEL_X && x<PANEL_X+416 && y>=PANEL_Y && y<PANEL_Y+96;
    wire [11:0] text_x=x-(PANEL_X+8),text_y=y-(PANEL_Y+8);
    reg [1:0] text_row;
    reg [3:0] font_y;
    reg row_valid;
    always @(*) begin
        text_row=0;font_y=0;row_valid=1;
        if(text_y<16) begin text_row=0;font_y=text_y[3:0];end
        else if(text_y>=22 && text_y<38) begin text_row=1;font_y=text_y-22;end
        else if(text_y>=44 && text_y<60) begin text_row=2;font_y=text_y-44;end
        else if(text_y>=66 && text_y<82) begin text_row=3;font_y=text_y-66;end
        else row_valid=0;
    end
    wire [7:0] ch=panel_char(text_row,text_x[8:4]);
    wire [15:0] font_bits;
    // The first pixel stage reads the character row from synchronous ERAM.
    // Delay its column and validity alongside the row; bit selection remains
    // in the second stage, preserving the existing two-cycle mixer alignment.
    osd_ascii_font_rom u_font(.I_clk(clk),.I_char(ch),.I_row(font_y),.O_row_bits(font_bits));
    reg panel_s1,letter_valid_s1,selected_s1,centre_s1;
    reg [3:0] font_column_s1;
    wire letter_s1=letter_valid_s1 && font_bits[font_column_s1];
    reg [1:0] row_s1,c_s1,state_s1;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            panel_s1<=0;letter_valid_s1<=0;font_column_s1<=0;
            selected_s1<=0;centre_s1<=0;row_s1<=0;c_s1<=0;state_s1<=0;
            hit<=0;colour<=0;
        end else begin
            panel_s1<=panel;
            letter_valid_s1<=panel && row_valid && x>=PANEL_X+8 && x<PANEL_X+408;
            font_column_s1<=4'd15-text_x[3:0];
            selected_s1<=selected;centre_s1<=centre_mark;row_s1<=text_row;c_s1<=frame_box[46:45];state_s1<=frame_state;
            hit<=panel_s1 || selected_s1 || centre_s1;
            if(panel_s1) begin
                if(!letter_s1) colour<=PANEL_BG;
                else if(row_s1==1) case(c_s1)
                    1:colour<=24'hff6060;2:colour<=24'h60ff80;3:colour<=24'h80aaff;default:colour<=TEXT;
                endcase
                else if(row_s1==0 && state_s1==LOST) colour<=24'hffb040;
                else if(row_s1==0 && state_s1==LOCKED) colour<=LOCK_COLOUR;
                else colour<=TEXT;
            end else if(selected_s1) colour<=LOCK_COLOUR;
            else if(centre_s1) colour<=CENTRE_COLOUR;
            else colour<=0;
        end
    end
endmodule
