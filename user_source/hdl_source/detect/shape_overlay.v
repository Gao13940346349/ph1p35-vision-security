// Nine coloured boxes and lowercase 16x16-cell labels, two pixel-clock stages.
// Coordinates/DE enter at the mixer input, outputs align with its existing DE_2d.
module shape_overlay #(
    parameter integer SCREEN_WIDTH=1280,
    parameter integer SCREEN_HEIGHT=720,
    parameter integer H_OFFSET=0,
    parameter integer V_OFFSET=0,
    parameter integer SHOW_COORDINATES=1,
    parameter integer BORDER_WIDTH=3,
    parameter integer APPLICATION_SUPPORT=0
)(
    input wire clk,rst_n,de,
    input wire application_mode,
    input wire [11:0] x,y,
    input wire [422:0] boxes,
    output reg hit,
    output reg [23:0] colour
);
    // Application annotations reuse the same two-row region and font banks.
    // A disabled parameter also preserves legacy stand-alone instances whose
    // application_mode input is intentionally not connected.
    wire application=APPLICATION_SUPPORT ? application_mode : 1'b0;
    // Compact glyph addresses, shared by each pair of synchronous ROM ports.
    // Pixel data never passes through a 64-bit ASCII-to-glyph selector.
    function [4:0] font_code;
        input [7:0] ch;
        begin
            case(ch)
                "a":font_code=0; "c":font_code=1; "e":font_code=2;
                "g":font_code=3; "i":font_code=4; "l":font_code=5;
                "n":font_code=6; "q":font_code=7; "r":font_code=8;
                "s":font_code=9; "t":font_code=10; "u":font_code=11;
                "h":font_code=12; "o":font_code=13; "p":font_code=14;
                "v":font_code=15;
                "0":font_code=16; "1":font_code=17; "2":font_code=18;
                "3":font_code=19; "4":font_code=20; "5":font_code=21;
                "6":font_code=22; "7":font_code=23; "8":font_code=24;
                "9":font_code=25; "(":font_code=26; ")":font_code=27;
                ",":font_code=28; default:font_code=31;
            endcase
        end
    endfunction
    function [7:0] application_letter;
        input [1:0] level,kind;
        input second_line;
        input [2:0] index;
        begin
            application_letter=0;
            if(second_line) case(kind)
                0:case(index)
                    0:application_letter="v";1:application_letter="e";
                    2:application_letter="h";3:application_letter="i";
                    4:application_letter="c";5:application_letter="l";
                    6:application_letter="e";
                endcase
                1:case(index)
                    0:application_letter="e";1:application_letter="q";
                    2:application_letter="u";3:application_letter="i";
                    4:application_letter="p";
                endcase
                2:case(index)
                    0:application_letter="p";1:application_letter="e";
                    2:application_letter="r";3:application_letter="s";
                    4:application_letter="o";5:application_letter="n";
                endcase
            endcase
            else case(level)
                1:case(index)
                    0:application_letter="r";1:application_letter="e";
                    2:application_letter="s";3:application_letter="t";
                    4:application_letter="r";5:application_letter="i";
                    6:application_letter="c";7:application_letter="t";
                endcase
                2:case(index)
                    0:application_letter="a";1:application_letter="u";
                    2:application_letter="t";3:application_letter="h";
                endcase
                3:case(index)
                    0:application_letter="v";1:application_letter="i";
                    2:application_letter="s";3:application_letter="i";
                    4:application_letter="t";5:application_letter="o";
                    6:application_letter="r";
                endcase
            endcase
        end
    endfunction
    function [7:0] letter;
        input [1:0] shape;
        input [2:0] index;
        begin
            letter=0;
            case(shape)
                0: case(index)
                    0:letter="t";1:letter="r";2:letter="i";3:letter="a";
                    4:letter="n";5:letter="g";6:letter="l";7:letter="e";
                endcase
                1: case(index)
                    0:letter="s";1:letter="q";2:letter="u";3:letter="a";
                    4:letter="r";5:letter="e";default:letter=0;
                endcase
                2: case(index)
                    0:letter="c";1:letter="i";2:letter="r";3:letter="c";
                    4:letter="l";5:letter="e";default:letter=0;
                endcase
                default:letter=0;
            endcase
        end
    endfunction
    // Shared sequential binary-to-decimal converter: no pixel-rate division,
    // and no replication of a converter for each of the nine objects.
    wire [287:0] centres;
    generate if(SHOW_COORDINATES) begin:g_coordinates
    reg [287:0] converted_centres;
    assign centres=converted_centres;
    reg [3:0] slot,step;
    reg [10:0] bx,by;
    reg [15:0] decimal_x,decimal_y;
    reg [15:0] adjusted_x,adjusted_y;
    wire [46:0] cb=boxes[slot*47 +:47];
    wire [11:0] sum_x={1'b0,cb[41:31]}+{1'b0,cb[30:20]};
    wire [10:0] sum_y={1'b0,cb[19:10]}+{1'b0,cb[9:0]};
    integer digit;
    always @(*) begin
        adjusted_x=decimal_x; adjusted_y=decimal_y;
        for(digit=0;digit<4;digit=digit+1) begin
            if(decimal_x[digit*4 +:4]>=5) adjusted_x[digit*4 +:4]=decimal_x[digit*4 +:4]+4'd3;
            if(decimal_y[digit*4 +:4]>=5) adjusted_y[digit*4 +:4]=decimal_y[digit*4 +:4]+4'd3;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin converted_centres<=0;slot<=0;step<=0;bx<=0;by<=0;decimal_x<=0;decimal_y<=0;end
        else if(step==0) begin
            bx<=sum_x[11:1]+H_OFFSET;by<={1'b0,sum_y[10:1]}+V_OFFSET;
            decimal_x<=0;decimal_y<=0;step<=1;
        end else if(step<=11) begin
            decimal_x<={adjusted_x[14:0],bx[10]};
            decimal_y<={adjusted_y[14:0],by[10]};
            bx<={bx[9:0],1'b0};by<={by[9:0],1'b0};step<=step+1'b1;
        end else begin
            converted_centres[slot*32 +:32]<={decimal_x,decimal_y};
            slot<=slot==8 ? 0 : slot+1'b1;step<=0;
        end
    end
    end else begin:g_no_coordinates
        assign centres=288'd0;
    end endgenerate
    function [7:0] coordinate_letter;
        input [31:0] value;
        input [3:0] index;
        begin
            case(index)
                0:coordinate_letter="(";
                1:coordinate_letter="0"+value[31:28];
                2:coordinate_letter="0"+value[27:24];
                3:coordinate_letter="0"+value[23:20];
                4:coordinate_letter="0"+value[19:16];
                5:coordinate_letter=",";
                6:coordinate_letter="0"+value[11:8];
                7:coordinate_letter="0"+value[7:4];
                8:coordinate_letter="0"+value[3:0];
                9:coordinate_letter=")";
                default:coordinate_letter=0;
            endcase
        end
    endfunction
    wire [8:0] border_s1,glyph_hit;
    wire [17:0] colours_s1;
    wire [79:0] font_addresses;
    wire [79:0] font_rows;
    genvar g;
    generate for(g=0;g<9;g=g+1) begin : object
        wire [46:0] b=boxes[g*47 +:47];
        wire [11:0] x0={1'b0,b[41:31]}+H_OFFSET;
        wire [11:0] x1={1'b0,b[30:20]}+H_OFFSET;
        wire [11:0] y0={2'd0,b[19:10]}+V_OFFSET;
        wire [11:0] y1={2'd0,b[9:0]}+V_OFFSET;
        wire [11:0] label_w=12'd160;
        wire [11:0] label_x=x0>SCREEN_WIDTH-label_w ? SCREEN_WIDTH-label_w : x0;
        wire [11:0] label_y=y0>=32 ? y0-12'd32 : y0+12'd3;
        reg border,text;
        wire [11:0] dx=x-label_x,dy=y-label_y;
        wire [7:0] ch=application ?
            (dx<128 ? application_letter(b[46:45],b[44:43],dy[4],dx[6:4]) : 8'd0) :
            (SHOW_COORDINATES && dy[4]) ? coordinate_letter(centres[g*32 +:32],dx[7:4]) :
            (dx<128 ? letter(b[44:43],dx[6:4]) : 8'd0);
        assign font_addresses[g*8 +:8]={font_code(ch),dy[3:1]};
        reg [2:0] column_s1;
        reg [1:0] c;
        // A one-pixel ordinary box uses equality tests instead of four
        // pixel-rate subtractors per object. Selected tracking stays thicker.
        wire border_edge=BORDER_WIDTH==1 ? (x==x0 || x==x1 || y==y0 || y==y1) :
            (x-x0<BORDER_WIDTH || x1-x<BORDER_WIDTH ||
             y-y0<BORDER_WIDTH || y1-y<BORDER_WIDTH);
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin border<=0;text<=0;column_s1<=0;c<=0;end
            else begin
                column_s1<=dx[3:1];c<=b[46:45];
                border<=de && b[42] && b[46:45]!=0 && b[44:43]<3 &&
                    x>=x0 && x<=x1 && y>=y0 && y<=y1 &&
                    border_edge;
                text<=de && b[42] && b[46:45]!=0 && b[44:43]<3 &&
                    x>=label_x && x<label_x+label_w && y>=label_y &&
                    y<label_y+((SHOW_COORDINATES || application) ? 32 : 16) &&
                    x<SCREEN_WIDTH && y<SCREEN_HEIGHT;
            end
        end
        wire [7:0] row_bits=font_rows[g*8 +:8];
        assign glyph_hit[g]=text && row_bits[3'd7-column_s1];
        assign border_s1[g]=border;
        assign colours_s1[g*2 +:2]=c;
    end endgenerate
    // Nine independent label reads, paired into five dual-read block ROMs.
    // The unused tenth port has a constant address and no output consumer.
    assign font_addresses[72 +:8]=8'd0;
    generate for(g=0;g<5;g=g+1) begin:font_bank
        shape_label_font u_rows(.clk(clk),
            .address_a(font_addresses[g*16 +:8]),
            .address_b(font_addresses[g*16+8 +:8]),
            .row_a(font_rows[g*16 +:8]),.row_b(font_rows[g*16+8 +:8]));
    end endgenerate
    reg found;
    reg [1:0] chosen;
    integer i;
    always @(*) begin
        chosen=0;found=0;
        // Letters take precedence over borders; lower slot wins overlaps.
        for(i=0;i<9;i=i+1) if(!found && glyph_hit[i]) begin
            found=1;chosen=colours_s1[i*2 +:2];
        end
        for(i=0;i<9;i=i+1) if(!found && border_s1[i]) begin
            found=1;chosen=colours_s1[i*2 +:2];
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin hit<=0;colour<=0;end
        else begin
            hit<=found;
            case(chosen)
                1:colour<=24'hff0000;
                2:colour<=24'h00ff00;
                3:colour<=24'h0000ff;
                default:colour<=24'd0;
            endcase
        end
    end
endmodule

// One immutable 256x8 table with two synchronous read ports. Initialization
// uses constants only; no font decoder or reset network is synthesized.
module shape_label_font(
    input wire clk,
    input wire [7:0] address_a,address_b,
    output reg [7:0] row_a,row_b
);
    (* ram_style="bram", rom_style="bram" *) reg [7:0] rows[0:255];
    function [63:0] glyph;
        input [4:0] code;
        begin
            case(code)
                0:glyph=64'h000038043c443c00; 1:glyph=64'h00003c4040403c00;
                2:glyph=64'h000038447c403c00; 3:glyph=64'h00003c44443c0478;
                4:glyph=64'h1000301010103800; 5:glyph=64'h3010101010103800;
                6:glyph=64'h0000586444444400; 7:glyph=64'h00003c44443c0406;
                8:glyph=64'h0000586440404000; 9:glyph=64'h00003c4038047800;
                10:glyph=64'h10103c1010141800; 11:glyph=64'h00004444444c3400;
                12:glyph=64'h0040407844444400; 13:glyph=64'h0000384444443800;
                14:glyph=64'h0000784444784040; 15:glyph=64'h0000444444281000;
                16:glyph=64'h0038444c54443800; 17:glyph=64'h0010301010103800;
                18:glyph=64'h0038440810207c00; 19:glyph=64'h0078083008443800;
                20:glyph=64'h00081828487c0800; 21:glyph=64'h007c407804443800;
                22:glyph=64'h0038407844443800; 23:glyph=64'h007c040810202000;
                24:glyph=64'h0038443844443800; 25:glyph=64'h003844443c043800;
                26:glyph=64'h0008102020100800; 27:glyph=64'h0020100808102000;
                28:glyph=64'h0000000000181830; default:glyph=0;
            endcase
        end
    endfunction
    integer a;
    initial for(a=0;a<256;a=a+1) rows[a]=glyph(a/8) >> (56-8*(a%8));
    always @(posedge clk) begin
        row_a<=rows[address_a];
        row_b<=rows[address_b];
    end
endmodule
