module osd_char_lib (
    input wire [7:0] I_char,
    input wire [3:0] I_row,
    output reg [15:0] O_row_bits
);

    localparam CHAR_AN    = 8'h80;
    localparam CHAR_LU    = 8'h81;
    localparam CHAR_A     = 8'h41;
    localparam CHAR_B     = 8'h42;
    localparam CHAR_C     = 8'h43;
    localparam CHAR_D     = 8'h44;
    localparam CHAR_E     = 8'h45;
    localparam CHAR_F     = 8'h46;
    localparam CHAR_G     = 8'h47;
    localparam CHAR_H     = 8'h48;
    localparam CHAR_I     = 8'h49;
    localparam CHAR_J     = 8'h4a;
    localparam CHAR_K     = 8'h4b;
    localparam CHAR_L     = 8'h4c;
    localparam CHAR_M     = 8'h4d;
    localparam CHAR_N     = 8'h4e;
    localparam CHAR_O     = 8'h4f;
    localparam CHAR_P     = 8'h50;
    localparam CHAR_Q     = 8'h51;
    localparam CHAR_R     = 8'h52;
    localparam CHAR_S     = 8'h53;
    localparam CHAR_T     = 8'h54;
    localparam CHAR_U     = 8'h55;
    localparam CHAR_V     = 8'h56;
    localparam CHAR_W     = 8'h57;
    localparam CHAR_X     = 8'h58;
    localparam CHAR_Y     = 8'h59;
    localparam CHAR_Z     = 8'h5a;
    localparam CHAR_0     = 8'h30;
    localparam CHAR_1     = 8'h31;
    localparam CHAR_2     = 8'h32;
    localparam CHAR_3     = 8'h33;
    localparam CHAR_4     = 8'h34;
    localparam CHAR_5     = 8'h35;
    localparam CHAR_6     = 8'h36;
    localparam CHAR_7     = 8'h37;
    localparam CHAR_8     = 8'h38;
    localparam CHAR_9     = 8'h39;
    localparam CHAR_COLON = 8'h3a;

    always @(*) begin
        case(I_char)
            8'h2e: O_row_bits=(I_row==10 || I_row==11) ? 16'h0180 : 16'h0000;
            8'h2d: O_row_bits=(I_row==6 || I_row==7) ? 16'h0ff0 : 16'h0000;
            8'h25: begin
                case(I_row)
                    1:O_row_bits=16'h0c18;
                    2:O_row_bits=16'h1220;
                    3:O_row_bits=16'h1240;
                    4:O_row_bits=16'h0c80;
                    5:O_row_bits=16'h0100;
                    6:O_row_bits=16'h0200;
                    7:O_row_bits=16'h0430;
                    8:O_row_bits=16'h0848;
                    9:O_row_bits=16'h1048;
                    10:O_row_bits=16'h2030;
                    default:O_row_bits=16'h0000;
                endcase
            end
            CHAR_AN: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0200;
                    4'd1: O_row_bits = 16'h7ff0;
                    4'd2: O_row_bits = 16'h4410;
                    4'd3: O_row_bits = 16'h0400;
                    4'd4: O_row_bits = 16'h7ff0;
                    4'd5: O_row_bits = 16'h0840;
                    4'd6: O_row_bits = 16'h1040;
                    4'd7: O_row_bits = 16'h1c80;
                    4'd8: O_row_bits = 16'h0300;
                    4'd9: O_row_bits = 16'h0cc0;
                    4'd10: O_row_bits = 16'h7020;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_LU: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h79e0;
                    4'd1: O_row_bits = 16'h4a20;
                    4'd2: O_row_bits = 16'h4d40;
                    4'd3: O_row_bits = 16'h7880;
                    4'd4: O_row_bits = 16'h1140;
                    4'd5: O_row_bits = 16'h1630;
                    4'd6: O_row_bits = 16'h5be0;
                    4'd7: O_row_bits = 16'h5220;
                    4'd8: O_row_bits = 16'h5220;
                    4'd9: O_row_bits = 16'h5be0;
                    4'd10: O_row_bits = 16'h6220;
                    4'd11: O_row_bits = 16'h0000;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            // Original geometric capitals: 12 pixels high, two-pixel main strokes.
            // Same 16x16 cells and baseline as the existing decimal digits.
            CHAR_A: begin
                case(I_row)
                    4'd1: O_row_bits = 16'h03c0;
                    4'd2: O_row_bits = 16'h07e0;
                    4'd3: O_row_bits = 16'h0c30;
                    4'd4: O_row_bits = 16'h1818;
                    4'd5: O_row_bits = 16'h300c;
                    4'd6: O_row_bits = 16'h300c;
                    4'd7: O_row_bits = 16'h3ffc;
                    4'd8: O_row_bits = 16'h3ffc;
                    4'd9: O_row_bits = 16'h300c;
                    4'd10: O_row_bits = 16'h300c;
                    4'd11: O_row_bits = 16'h300c;
                    4'd12: O_row_bits = 16'h300c;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_B: begin
                case(I_row)
                    4'd1: O_row_bits = 16'h3ff0;
                    4'd2: O_row_bits = 16'h3ff8;
                    4'd3: O_row_bits = 16'h301c;
                    4'd4: O_row_bits = 16'h300c;
                    4'd5: O_row_bits = 16'h301c;
                    4'd6: O_row_bits = 16'h3ff8;
                    4'd7: O_row_bits = 16'h3ff8;
                    4'd8: O_row_bits = 16'h301c;
                    4'd9: O_row_bits = 16'h300c;
                    4'd10: O_row_bits = 16'h301c;
                    4'd11: O_row_bits = 16'h3ff8;
                    4'd12: O_row_bits = 16'h3ff0;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_C: begin
                case(I_row)
                    4'd1: O_row_bits = 16'h0ff0;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h381c;
                    4'd4: O_row_bits = 16'h300c;
                    4'd5: O_row_bits = 16'h3000;
                    4'd6: O_row_bits = 16'h3000;
                    4'd7: O_row_bits = 16'h3000;
                    4'd8: O_row_bits = 16'h3000;
                    4'd9: O_row_bits = 16'h300c;
                    4'd10: O_row_bits = 16'h381c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h0ff0;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_D: begin
                case(I_row)
                    4'd1: O_row_bits = 16'h3fc0;
                    4'd2: O_row_bits = 16'h3fe0;
                    4'd3: O_row_bits = 16'h3070;
                    4'd4: O_row_bits = 16'h3038;
                    4'd5: O_row_bits = 16'h301c;
                    4'd6: O_row_bits = 16'h300c;
                    4'd7: O_row_bits = 16'h300c;
                    4'd8: O_row_bits = 16'h301c;
                    4'd9: O_row_bits = 16'h3038;
                    4'd10: O_row_bits = 16'h3070;
                    4'd11: O_row_bits = 16'h3fe0;
                    4'd12: O_row_bits = 16'h3fc0;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_E: begin
                case(I_row)
                    4'd1: O_row_bits = 16'h3ffc;
                    4'd2: O_row_bits = 16'h3ffc;
                    4'd3: O_row_bits = 16'h3000;
                    4'd4: O_row_bits = 16'h3000;
                    4'd5: O_row_bits = 16'h3000;
                    4'd6: O_row_bits = 16'h3ff0;
                    4'd7: O_row_bits = 16'h3ff0;
                    4'd8: O_row_bits = 16'h3000;
                    4'd9: O_row_bits = 16'h3000;
                    4'd10: O_row_bits = 16'h3000;
                    4'd11: O_row_bits = 16'h3ffc;
                    4'd12: O_row_bits = 16'h3ffc;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_F: begin
                case(I_row)
                    4'd1: O_row_bits = 16'h3ffc;
                    4'd2: O_row_bits = 16'h3ffc;
                    4'd3: O_row_bits = 16'h3000;
                    4'd4: O_row_bits = 16'h3000;
                    4'd5: O_row_bits = 16'h3000;
                    4'd6: O_row_bits = 16'h3ff0;
                    4'd7: O_row_bits = 16'h3ff0;
                    4'd8: O_row_bits = 16'h3000;
                    4'd9: O_row_bits = 16'h3000;
                    4'd10: O_row_bits = 16'h3000;
                    4'd11: O_row_bits = 16'h3000;
                    4'd12: O_row_bits = 16'h3000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_G: begin
                case(I_row)
                    4'd1: O_row_bits = 16'h0ff0;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h381c;
                    4'd4: O_row_bits = 16'h300c;
                    4'd5: O_row_bits = 16'h3000;
                    4'd6: O_row_bits = 16'h3000;
                    4'd7: O_row_bits = 16'h30fc;
                    4'd8: O_row_bits = 16'h30fc;
                    4'd9: O_row_bits = 16'h300c;
                    4'd10: O_row_bits = 16'h381c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h0ff0;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_H: begin
                case(I_row)
                    4'd1: O_row_bits = 16'h300c;
                    4'd2: O_row_bits = 16'h300c;
                    4'd3: O_row_bits = 16'h300c;
                    4'd4: O_row_bits = 16'h300c;
                    4'd5: O_row_bits = 16'h300c;
                    4'd6: O_row_bits = 16'h3ffc;
                    4'd7: O_row_bits = 16'h3ffc;
                    4'd8: O_row_bits = 16'h300c;
                    4'd9: O_row_bits = 16'h300c;
                    4'd10: O_row_bits = 16'h300c;
                    4'd11: O_row_bits = 16'h300c;
                    4'd12: O_row_bits = 16'h300c;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_I: begin
                case(I_row)
                    4'd1: O_row_bits = 16'h0ff0;
                    4'd2: O_row_bits = 16'h0ff0;
                    4'd3: O_row_bits = 16'h0180;
                    4'd4: O_row_bits = 16'h0180;
                    4'd5: O_row_bits = 16'h0180;
                    4'd6: O_row_bits = 16'h0180;
                    4'd7: O_row_bits = 16'h0180;
                    4'd8: O_row_bits = 16'h0180;
                    4'd9: O_row_bits = 16'h0180;
                    4'd10: O_row_bits = 16'h0180;
                    4'd11: O_row_bits = 16'h0ff0;
                    4'd12: O_row_bits = 16'h0ff0;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_J: begin
                case(I_row)
                    4'd1: O_row_bits = 16'h00fc;
                    4'd2: O_row_bits = 16'h00fc;
                    4'd3: O_row_bits = 16'h0018;
                    4'd4: O_row_bits = 16'h0018;
                    4'd5: O_row_bits = 16'h0018;
                    4'd6: O_row_bits = 16'h0018;
                    4'd7: O_row_bits = 16'h0018;
                    4'd8: O_row_bits = 16'h3018;
                    4'd9: O_row_bits = 16'h3018;
                    4'd10: O_row_bits = 16'h3838;
                    4'd11: O_row_bits = 16'h1ff0;
                    4'd12: O_row_bits = 16'h0fe0;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_K: begin
                case(I_row)
                    4'd1: O_row_bits = 16'h300c;
                    4'd2: O_row_bits = 16'h301c;
                    4'd3: O_row_bits = 16'h3038;
                    4'd4: O_row_bits = 16'h3070;
                    4'd5: O_row_bits = 16'h30e0;
                    4'd6: O_row_bits = 16'h31c0;
                    4'd7: O_row_bits = 16'h3380;
                    4'd8: O_row_bits = 16'h31c0;
                    4'd9: O_row_bits = 16'h30e0;
                    4'd10: O_row_bits = 16'h3070;
                    4'd11: O_row_bits = 16'h3038;
                    4'd12: O_row_bits = 16'h301c;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_L: begin
                case(I_row)
                    4'd1: O_row_bits = 16'h3000;
                    4'd2: O_row_bits = 16'h3000;
                    4'd3: O_row_bits = 16'h3000;
                    4'd4: O_row_bits = 16'h3000;
                    4'd5: O_row_bits = 16'h3000;
                    4'd6: O_row_bits = 16'h3000;
                    4'd7: O_row_bits = 16'h3000;
                    4'd8: O_row_bits = 16'h3000;
                    4'd9: O_row_bits = 16'h3000;
                    4'd10: O_row_bits = 16'h3000;
                    4'd11: O_row_bits = 16'h3ffc;
                    4'd12: O_row_bits = 16'h3ffc;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_M: begin
                case(I_row)
                    4'd1: O_row_bits = 16'h300c;
                    4'd2: O_row_bits = 16'h381c;
                    4'd3: O_row_bits = 16'h3c3c;
                    4'd4: O_row_bits = 16'h366c;
                    4'd5: O_row_bits = 16'h33cc;
                    4'd6: O_row_bits = 16'h318c;
                    4'd7: O_row_bits = 16'h300c;
                    4'd8: O_row_bits = 16'h300c;
                    4'd9: O_row_bits = 16'h300c;
                    4'd10: O_row_bits = 16'h300c;
                    4'd11: O_row_bits = 16'h300c;
                    4'd12: O_row_bits = 16'h300c;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_N: begin
                case(I_row)
                    4'd1: O_row_bits = 16'h300c;
                    4'd2: O_row_bits = 16'h380c;
                    4'd3: O_row_bits = 16'h3c0c;
                    4'd4: O_row_bits = 16'h360c;
                    4'd5: O_row_bits = 16'h330c;
                    4'd6: O_row_bits = 16'h318c;
                    4'd7: O_row_bits = 16'h30cc;
                    4'd8: O_row_bits = 16'h306c;
                    4'd9: O_row_bits = 16'h303c;
                    4'd10: O_row_bits = 16'h301c;
                    4'd11: O_row_bits = 16'h300c;
                    4'd12: O_row_bits = 16'h300c;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_O: begin
                case(I_row)
                    4'd1: O_row_bits = 16'h0ff0;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h381c;
                    4'd4: O_row_bits = 16'h300c;
                    4'd5: O_row_bits = 16'h300c;
                    4'd6: O_row_bits = 16'h300c;
                    4'd7: O_row_bits = 16'h300c;
                    4'd8: O_row_bits = 16'h300c;
                    4'd9: O_row_bits = 16'h300c;
                    4'd10: O_row_bits = 16'h381c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h0ff0;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_P: begin
                case(I_row)
                    4'd1: O_row_bits = 16'h3ff0;
                    4'd2: O_row_bits = 16'h3ff8;
                    4'd3: O_row_bits = 16'h301c;
                    4'd4: O_row_bits = 16'h300c;
                    4'd5: O_row_bits = 16'h300c;
                    4'd6: O_row_bits = 16'h301c;
                    4'd7: O_row_bits = 16'h3ff8;
                    4'd8: O_row_bits = 16'h3ff0;
                    4'd9: O_row_bits = 16'h3000;
                    4'd10: O_row_bits = 16'h3000;
                    4'd11: O_row_bits = 16'h3000;
                    4'd12: O_row_bits = 16'h3000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_Q: begin
                case(I_row)
                    4'd1: O_row_bits = 16'h0ff0;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h381c;
                    4'd4: O_row_bits = 16'h300c;
                    4'd5: O_row_bits = 16'h300c;
                    4'd6: O_row_bits = 16'h300c;
                    4'd7: O_row_bits = 16'h300c;
                    4'd8: O_row_bits = 16'h30cc;
                    4'd9: O_row_bits = 16'h306c;
                    4'd10: O_row_bits = 16'h3838;
                    4'd11: O_row_bits = 16'h1ffc;
                    4'd12: O_row_bits = 16'h0fec;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_R: begin
                case(I_row)
                    4'd1: O_row_bits = 16'h3ff0;
                    4'd2: O_row_bits = 16'h3ff8;
                    4'd3: O_row_bits = 16'h301c;
                    4'd4: O_row_bits = 16'h300c;
                    4'd5: O_row_bits = 16'h300c;
                    4'd6: O_row_bits = 16'h301c;
                    4'd7: O_row_bits = 16'h3ff8;
                    4'd8: O_row_bits = 16'h3ff0;
                    4'd9: O_row_bits = 16'h30e0;
                    4'd10: O_row_bits = 16'h3070;
                    4'd11: O_row_bits = 16'h3038;
                    4'd12: O_row_bits = 16'h301c;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_S: begin
                case(I_row)
                    4'd1: O_row_bits = 16'h0ff0;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h381c;
                    4'd4: O_row_bits = 16'h3000;
                    4'd5: O_row_bits = 16'h3800;
                    4'd6: O_row_bits = 16'h1fe0;
                    4'd7: O_row_bits = 16'h07f8;
                    4'd8: O_row_bits = 16'h001c;
                    4'd9: O_row_bits = 16'h000c;
                    4'd10: O_row_bits = 16'h381c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h0ff0;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_T: begin
                case(I_row)
                    4'd1: O_row_bits = 16'h3ffc;
                    4'd2: O_row_bits = 16'h3ffc;
                    4'd3: O_row_bits = 16'h0180;
                    4'd4: O_row_bits = 16'h0180;
                    4'd5: O_row_bits = 16'h0180;
                    4'd6: O_row_bits = 16'h0180;
                    4'd7: O_row_bits = 16'h0180;
                    4'd8: O_row_bits = 16'h0180;
                    4'd9: O_row_bits = 16'h0180;
                    4'd10: O_row_bits = 16'h0180;
                    4'd11: O_row_bits = 16'h0180;
                    4'd12: O_row_bits = 16'h0180;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_U: begin
                case(I_row)
                    4'd1: O_row_bits = 16'h300c;
                    4'd2: O_row_bits = 16'h300c;
                    4'd3: O_row_bits = 16'h300c;
                    4'd4: O_row_bits = 16'h300c;
                    4'd5: O_row_bits = 16'h300c;
                    4'd6: O_row_bits = 16'h300c;
                    4'd7: O_row_bits = 16'h300c;
                    4'd8: O_row_bits = 16'h300c;
                    4'd9: O_row_bits = 16'h300c;
                    4'd10: O_row_bits = 16'h381c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h0ff0;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_V: begin
                case(I_row)
                    4'd1: O_row_bits = 16'h300c;
                    4'd2: O_row_bits = 16'h300c;
                    4'd3: O_row_bits = 16'h300c;
                    4'd4: O_row_bits = 16'h300c;
                    4'd5: O_row_bits = 16'h300c;
                    4'd6: O_row_bits = 16'h1818;
                    4'd7: O_row_bits = 16'h1818;
                    4'd8: O_row_bits = 16'h0c30;
                    4'd9: O_row_bits = 16'h0c30;
                    4'd10: O_row_bits = 16'h0660;
                    4'd11: O_row_bits = 16'h03c0;
                    4'd12: O_row_bits = 16'h0180;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_W: begin
                case(I_row)
                    4'd1: O_row_bits = 16'h300c;
                    4'd2: O_row_bits = 16'h300c;
                    4'd3: O_row_bits = 16'h300c;
                    4'd4: O_row_bits = 16'h300c;
                    4'd5: O_row_bits = 16'h300c;
                    4'd6: O_row_bits = 16'h300c;
                    4'd7: O_row_bits = 16'h318c;
                    4'd8: O_row_bits = 16'h33cc;
                    4'd9: O_row_bits = 16'h366c;
                    4'd10: O_row_bits = 16'h3c3c;
                    4'd11: O_row_bits = 16'h381c;
                    4'd12: O_row_bits = 16'h300c;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_X: begin
                case(I_row)
                    4'd1: O_row_bits = 16'h300c;
                    4'd2: O_row_bits = 16'h381c;
                    4'd3: O_row_bits = 16'h1c38;
                    4'd4: O_row_bits = 16'h0e70;
                    4'd5: O_row_bits = 16'h07e0;
                    4'd6: O_row_bits = 16'h03c0;
                    4'd7: O_row_bits = 16'h03c0;
                    4'd8: O_row_bits = 16'h07e0;
                    4'd9: O_row_bits = 16'h0e70;
                    4'd10: O_row_bits = 16'h1c38;
                    4'd11: O_row_bits = 16'h381c;
                    4'd12: O_row_bits = 16'h300c;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_Y: begin
                case(I_row)
                    4'd1: O_row_bits = 16'h300c;
                    4'd2: O_row_bits = 16'h300c;
                    4'd3: O_row_bits = 16'h1818;
                    4'd4: O_row_bits = 16'h0c30;
                    4'd5: O_row_bits = 16'h0660;
                    4'd6: O_row_bits = 16'h03c0;
                    4'd7: O_row_bits = 16'h0180;
                    4'd8: O_row_bits = 16'h0180;
                    4'd9: O_row_bits = 16'h0180;
                    4'd10: O_row_bits = 16'h0180;
                    4'd11: O_row_bits = 16'h0180;
                    4'd12: O_row_bits = 16'h0180;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_Z: begin
                case(I_row)
                    4'd1: O_row_bits = 16'h3ffc;
                    4'd2: O_row_bits = 16'h3ffc;
                    4'd3: O_row_bits = 16'h001c;
                    4'd4: O_row_bits = 16'h0038;
                    4'd5: O_row_bits = 16'h0070;
                    4'd6: O_row_bits = 16'h00e0;
                    4'd7: O_row_bits = 16'h01c0;
                    4'd8: O_row_bits = 16'h0380;
                    4'd9: O_row_bits = 16'h0700;
                    4'd10: O_row_bits = 16'h0e00;
                    4'd11: O_row_bits = 16'h3ffc;
                    4'd12: O_row_bits = 16'h3ffc;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_0: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h1ff8;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h300c;
                    4'd4: O_row_bits = 16'h300c;
                    4'd5: O_row_bits = 16'h300c;
                    4'd6: O_row_bits = 16'h0000;
                    4'd7: O_row_bits = 16'h0000;
                    4'd8: O_row_bits = 16'h300c;
                    4'd9: O_row_bits = 16'h300c;
                    4'd10: O_row_bits = 16'h300c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_1: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h000c;
                    4'd2: O_row_bits = 16'h000c;
                    4'd3: O_row_bits = 16'h000c;
                    4'd4: O_row_bits = 16'h000c;
                    4'd5: O_row_bits = 16'h000c;
                    4'd6: O_row_bits = 16'h0000;
                    4'd7: O_row_bits = 16'h0000;
                    4'd8: O_row_bits = 16'h000c;
                    4'd9: O_row_bits = 16'h000c;
                    4'd10: O_row_bits = 16'h000c;
                    4'd11: O_row_bits = 16'h000c;
                    4'd12: O_row_bits = 16'h000c;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_2: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h1ff8;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h000c;
                    4'd4: O_row_bits = 16'h000c;
                    4'd5: O_row_bits = 16'h000c;
                    4'd6: O_row_bits = 16'h1ff8;
                    4'd7: O_row_bits = 16'h1ff8;
                    4'd8: O_row_bits = 16'h3000;
                    4'd9: O_row_bits = 16'h3000;
                    4'd10: O_row_bits = 16'h3000;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_3: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h1ff8;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h000c;
                    4'd4: O_row_bits = 16'h000c;
                    4'd5: O_row_bits = 16'h000c;
                    4'd6: O_row_bits = 16'h1ff8;
                    4'd7: O_row_bits = 16'h1ff8;
                    4'd8: O_row_bits = 16'h000c;
                    4'd9: O_row_bits = 16'h000c;
                    4'd10: O_row_bits = 16'h000c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_4: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h300c;
                    4'd2: O_row_bits = 16'h300c;
                    4'd3: O_row_bits = 16'h300c;
                    4'd4: O_row_bits = 16'h300c;
                    4'd5: O_row_bits = 16'h300c;
                    4'd6: O_row_bits = 16'h1ff8;
                    4'd7: O_row_bits = 16'h1ff8;
                    4'd8: O_row_bits = 16'h000c;
                    4'd9: O_row_bits = 16'h000c;
                    4'd10: O_row_bits = 16'h000c;
                    4'd11: O_row_bits = 16'h000c;
                    4'd12: O_row_bits = 16'h000c;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_5: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h1ff8;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h3000;
                    4'd4: O_row_bits = 16'h3000;
                    4'd5: O_row_bits = 16'h3000;
                    4'd6: O_row_bits = 16'h1ff8;
                    4'd7: O_row_bits = 16'h1ff8;
                    4'd8: O_row_bits = 16'h000c;
                    4'd9: O_row_bits = 16'h000c;
                    4'd10: O_row_bits = 16'h000c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_6: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h1ff8;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h3000;
                    4'd4: O_row_bits = 16'h3000;
                    4'd5: O_row_bits = 16'h3000;
                    4'd6: O_row_bits = 16'h1ff8;
                    4'd7: O_row_bits = 16'h1ff8;
                    4'd8: O_row_bits = 16'h300c;
                    4'd9: O_row_bits = 16'h300c;
                    4'd10: O_row_bits = 16'h300c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_7: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h1ff8;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h000c;
                    4'd4: O_row_bits = 16'h000c;
                    4'd5: O_row_bits = 16'h000c;
                    4'd6: O_row_bits = 16'h0000;
                    4'd7: O_row_bits = 16'h0000;
                    4'd8: O_row_bits = 16'h000c;
                    4'd9: O_row_bits = 16'h000c;
                    4'd10: O_row_bits = 16'h000c;
                    4'd11: O_row_bits = 16'h000c;
                    4'd12: O_row_bits = 16'h000c;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_8: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h1ff8;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h300c;
                    4'd4: O_row_bits = 16'h300c;
                    4'd5: O_row_bits = 16'h300c;
                    4'd6: O_row_bits = 16'h1ff8;
                    4'd7: O_row_bits = 16'h1ff8;
                    4'd8: O_row_bits = 16'h300c;
                    4'd9: O_row_bits = 16'h300c;
                    4'd10: O_row_bits = 16'h300c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_9: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h1ff8;
                    4'd2: O_row_bits = 16'h1ff8;
                    4'd3: O_row_bits = 16'h300c;
                    4'd4: O_row_bits = 16'h300c;
                    4'd5: O_row_bits = 16'h300c;
                    4'd6: O_row_bits = 16'h1ff8;
                    4'd7: O_row_bits = 16'h1ff8;
                    4'd8: O_row_bits = 16'h000c;
                    4'd9: O_row_bits = 16'h000c;
                    4'd10: O_row_bits = 16'h000c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_COLON: begin
                case(I_row)
                    4'd0: O_row_bits = 16'h0000;
                    4'd1: O_row_bits = 16'h0000;
                    4'd2: O_row_bits = 16'h0000;
                    4'd3: O_row_bits = 16'h0000;
                    4'd4: O_row_bits = 16'h0000;
                    4'd5: O_row_bits = 16'h0000;
                    4'd6: O_row_bits = 16'h0180;
                    4'd7: O_row_bits = 16'h0180;
                    4'd8: O_row_bits = 16'h0000;
                    4'd9: O_row_bits = 16'h0000;
                    4'd10: O_row_bits = 16'h0180;
                    4'd11: O_row_bits = 16'h0180;
                    4'd12: O_row_bits = 16'h0000;
                    4'd13: O_row_bits = 16'h0000;
                    4'd14: O_row_bits = 16'h0000;
                    4'd15: O_row_bits = 16'h0000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            default: O_row_bits = 16'h0000;
        endcase
    end

endmodule

// Synchronous ASCII subset used by the application and tracking panels.
// The 1024 x 16 table fits one 20-Kbit ERAM; no external initialization file.
// Row lookup and pixel bit selection occupy separate pixel clocks.
module osd_ascii_font_rom (
    input wire I_clk,
    input wire [7:0] I_char,
    input wire [3:0] I_row,
    output reg [15:0] O_row_bits
);
    (* ram_style="bram", rom_style="bram" *) reg [15:0] rows [0:1023];
    function [255:0] glyph;
        input [5:0] code;
        begin
            case(code)
                6'h25:glyph=256'h00000c18122012400c8001000200043008481048203000000000000000000000; // ASCII 25
                6'h2d:glyph=256'h0000000000000000000000000ff00ff000000000000000000000000000000000; // ASCII 2d
                6'h2e:glyph=256'h0000000000000000000000000000000000000000018001800000000000000000; // ASCII 2e
                6'h30:glyph=256'h00001ff81ff8300c300c300c00000000300c300c300c1ff81ff8000000000000; // ASCII 30
                6'h31:glyph=256'h0000000c000c000c000c000c00000000000c000c000c000c000c000000000000; // ASCII 31
                6'h32:glyph=256'h00001ff81ff8000c000c000c1ff81ff83000300030001ff81ff8000000000000; // ASCII 32
                6'h33:glyph=256'h00001ff81ff8000c000c000c1ff81ff8000c000c000c1ff81ff8000000000000; // ASCII 33
                6'h34:glyph=256'h0000300c300c300c300c300c1ff81ff8000c000c000c000c000c000000000000; // ASCII 34
                6'h35:glyph=256'h00001ff81ff83000300030001ff81ff8000c000c000c1ff81ff8000000000000; // ASCII 35
                6'h36:glyph=256'h00001ff81ff83000300030001ff81ff8300c300c300c1ff81ff8000000000000; // ASCII 36
                6'h37:glyph=256'h00001ff81ff8000c000c000c00000000000c000c000c000c000c000000000000; // ASCII 37
                6'h38:glyph=256'h00001ff81ff8300c300c300c1ff81ff8300c300c300c1ff81ff8000000000000; // ASCII 38
                6'h39:glyph=256'h00001ff81ff8300c300c300c1ff81ff8000c000c000c1ff81ff8000000000000; // ASCII 39
                6'h3a:glyph=256'h0000000000000000000000000180018000000000018001800000000000000000; // ASCII 3a
                6'h01:glyph=256'h000003c007e00c301818300c300c3ffc3ffc300c300c300c300c000000000000; // ASCII 41
                6'h02:glyph=256'h00003ff03ff8301c300c301c3ff83ff8301c300c301c3ff83ff0000000000000; // ASCII 42
                6'h03:glyph=256'h00000ff01ff8381c300c3000300030003000300c381c1ff80ff0000000000000; // ASCII 43
                6'h04:glyph=256'h00003fc03fe030703038301c300c300c301c303830703fe03fc0000000000000; // ASCII 44
                6'h05:glyph=256'h00003ffc3ffc3000300030003ff03ff03000300030003ffc3ffc000000000000; // ASCII 45
                6'h06:glyph=256'h00003ffc3ffc3000300030003ff03ff030003000300030003000000000000000; // ASCII 46
                6'h07:glyph=256'h00000ff01ff8381c300c3000300030fc30fc300c381c1ff80ff0000000000000; // ASCII 47
                6'h08:glyph=256'h0000300c300c300c300c300c3ffc3ffc300c300c300c300c300c000000000000; // ASCII 48
                6'h09:glyph=256'h00000ff00ff0018001800180018001800180018001800ff00ff0000000000000; // ASCII 49
                6'h0a:glyph=256'h000000fc00fc001800180018001800183018301838381ff00fe0000000000000; // ASCII 4a
                6'h0b:glyph=256'h0000300c301c3038307030e031c0338031c030e030703038301c000000000000; // ASCII 4b
                6'h0c:glyph=256'h000030003000300030003000300030003000300030003ffc3ffc000000000000; // ASCII 4c
                6'h0d:glyph=256'h0000300c381c3c3c366c33cc318c300c300c300c300c300c300c000000000000; // ASCII 4d
                6'h0e:glyph=256'h0000300c380c3c0c360c330c318c30cc306c303c301c300c300c000000000000; // ASCII 4e
                6'h0f:glyph=256'h00000ff01ff8381c300c300c300c300c300c300c381c1ff80ff0000000000000; // ASCII 4f
                6'h10:glyph=256'h00003ff03ff8301c300c300c301c3ff83ff03000300030003000000000000000; // ASCII 50
                6'h11:glyph=256'h00000ff01ff8381c300c300c300c300c30cc306c38381ffc0fec000000000000; // ASCII 51
                6'h12:glyph=256'h00003ff03ff8301c300c300c301c3ff83ff030e030703038301c000000000000; // ASCII 52
                6'h13:glyph=256'h00000ff01ff8381c300038001fe007f8001c000c381c1ff80ff0000000000000; // ASCII 53
                6'h14:glyph=256'h00003ffc3ffc0180018001800180018001800180018001800180000000000000; // ASCII 54
                6'h15:glyph=256'h0000300c300c300c300c300c300c300c300c300c381c1ff80ff0000000000000; // ASCII 55
                6'h16:glyph=256'h0000300c300c300c300c300c181818180c300c30066003c00180000000000000; // ASCII 56
                6'h17:glyph=256'h0000300c300c300c300c300c300c318c33cc366c3c3c381c300c000000000000; // ASCII 57
                6'h18:glyph=256'h0000300c381c1c380e7007e003c003c007e00e701c38381c300c000000000000; // ASCII 58
                6'h19:glyph=256'h0000300c300c18180c30066003c0018001800180018001800180000000000000; // ASCII 59
                6'h1a:glyph=256'h00003ffc3ffc001c0038007000e001c0038007000e003ffc3ffc000000000000; // ASCII 5a
                default:glyph=256'd0;
            endcase
        end
    endfunction
    integer a;
    initial begin
        for(a=0;a<1024;a=a+1)
            rows[a]=glyph(a/16) >> (240-16*(a%16));
    end
    wire [5:0] code=(I_char>=8'd32 && I_char<8'd96) ? I_char[5:0] : 6'd32;
    always @(posedge I_clk) O_row_bits<=rows[{code,I_row}];
endmodule



// Two independent mixer glyph streams share one 1024 x 16 true-dual-port ROM.
// Prefix transparency must not suppress a separate calibration glyph lookup.
module osd_mixer_font_rom (
    input wire I_clk,
    input wire [7:0] I_char_a,I_char_b,
    input wire [3:0] I_row_a,I_row_b,
    output reg [15:0] O_row_a,O_row_b
);
    (* ram_style="bram", rom_style="bram" *) reg [15:0] rows [0:1023];
    function [255:0] glyph;
        input [5:0] code;
        begin
            case(code)
                6'h25:glyph=256'h00000c18122012400c8001000200043008481048203000000000000000000000; // ASCII 25
                6'h2d:glyph=256'h0000000000000000000000000ff00ff000000000000000000000000000000000; // ASCII 2d
                6'h2e:glyph=256'h0000000000000000000000000000000000000000018001800000000000000000; // ASCII 2e
                6'h30:glyph=256'h00001ff81ff8300c300c300c00000000300c300c300c1ff81ff8000000000000; // ASCII 30
                6'h31:glyph=256'h0000000c000c000c000c000c00000000000c000c000c000c000c000000000000; // ASCII 31
                6'h32:glyph=256'h00001ff81ff8000c000c000c1ff81ff83000300030001ff81ff8000000000000; // ASCII 32
                6'h33:glyph=256'h00001ff81ff8000c000c000c1ff81ff8000c000c000c1ff81ff8000000000000; // ASCII 33
                6'h34:glyph=256'h0000300c300c300c300c300c1ff81ff8000c000c000c000c000c000000000000; // ASCII 34
                6'h35:glyph=256'h00001ff81ff83000300030001ff81ff8000c000c000c1ff81ff8000000000000; // ASCII 35
                6'h36:glyph=256'h00001ff81ff83000300030001ff81ff8300c300c300c1ff81ff8000000000000; // ASCII 36
                6'h37:glyph=256'h00001ff81ff8000c000c000c00000000000c000c000c000c000c000000000000; // ASCII 37
                6'h38:glyph=256'h00001ff81ff8300c300c300c1ff81ff8300c300c300c1ff81ff8000000000000; // ASCII 38
                6'h39:glyph=256'h00001ff81ff8300c300c300c1ff81ff8000c000c000c1ff81ff8000000000000; // ASCII 39
                6'h3a:glyph=256'h0000000000000000000000000180018000000000018001800000000000000000; // ASCII 3a
                6'h01:glyph=256'h000003c007e00c301818300c300c3ffc3ffc300c300c300c300c000000000000; // ASCII 41
                6'h02:glyph=256'h00003ff03ff8301c300c301c3ff83ff8301c300c301c3ff83ff0000000000000; // ASCII 42
                6'h03:glyph=256'h00000ff01ff8381c300c3000300030003000300c381c1ff80ff0000000000000; // ASCII 43
                6'h04:glyph=256'h00003fc03fe030703038301c300c300c301c303830703fe03fc0000000000000; // ASCII 44
                6'h05:glyph=256'h00003ffc3ffc3000300030003ff03ff03000300030003ffc3ffc000000000000; // ASCII 45
                6'h06:glyph=256'h00003ffc3ffc3000300030003ff03ff030003000300030003000000000000000; // ASCII 46
                6'h07:glyph=256'h00000ff01ff8381c300c3000300030fc30fc300c381c1ff80ff0000000000000; // ASCII 47
                6'h08:glyph=256'h0000300c300c300c300c300c3ffc3ffc300c300c300c300c300c000000000000; // ASCII 48
                6'h09:glyph=256'h00000ff00ff0018001800180018001800180018001800ff00ff0000000000000; // ASCII 49
                6'h0a:glyph=256'h000000fc00fc001800180018001800183018301838381ff00fe0000000000000; // ASCII 4a
                6'h0b:glyph=256'h0000300c301c3038307030e031c0338031c030e030703038301c000000000000; // ASCII 4b
                6'h0c:glyph=256'h000030003000300030003000300030003000300030003ffc3ffc000000000000; // ASCII 4c
                6'h0d:glyph=256'h0000300c381c3c3c366c33cc318c300c300c300c300c300c300c000000000000; // ASCII 4d
                6'h0e:glyph=256'h0000300c380c3c0c360c330c318c30cc306c303c301c300c300c000000000000; // ASCII 4e
                6'h0f:glyph=256'h00000ff01ff8381c300c300c300c300c300c300c381c1ff80ff0000000000000; // ASCII 4f
                6'h10:glyph=256'h00003ff03ff8301c300c300c301c3ff83ff03000300030003000000000000000; // ASCII 50
                6'h11:glyph=256'h00000ff01ff8381c300c300c300c300c30cc306c38381ffc0fec000000000000; // ASCII 51
                6'h12:glyph=256'h00003ff03ff8301c300c300c301c3ff83ff030e030703038301c000000000000; // ASCII 52
                6'h13:glyph=256'h00000ff01ff8381c300038001fe007f8001c000c381c1ff80ff0000000000000; // ASCII 53
                6'h14:glyph=256'h00003ffc3ffc0180018001800180018001800180018001800180000000000000; // ASCII 54
                6'h15:glyph=256'h0000300c300c300c300c300c300c300c300c300c381c1ff80ff0000000000000; // ASCII 55
                6'h16:glyph=256'h0000300c300c300c300c300c181818180c300c30066003c00180000000000000; // ASCII 56
                6'h17:glyph=256'h0000300c300c300c300c300c300c318c33cc366c3c3c381c300c000000000000; // ASCII 57
                6'h18:glyph=256'h0000300c381c1c380e7007e003c003c007e00e701c38381c300c000000000000; // ASCII 58
                6'h19:glyph=256'h0000300c300c18180c30066003c0018001800180018001800180000000000000; // ASCII 59
                6'h1a:glyph=256'h00003ffc3ffc001c0038007000e001c0038007000e003ffc3ffc000000000000; // ASCII 5a
                6'h00:glyph=256'h02007ff0441004007ff0084010401c8003000cc0702000000000000000000000; // Chinese AN
                6'h1b:glyph=256'h79e04a204d407880114016305be0522052205be0622000000000000000000000; // Chinese LU
                default:glyph=256'd0;
            endcase
        end
    endfunction
    function [5:0] font_code;
        input [7:0] character;
        begin
            if(character==8'h80)font_code=6'h00;
            else if(character==8'h81)font_code=6'h1b;
            // '@' and '[' were blank in the original library. Their table
            // slots hold the two Chinese glyphs; preserve ASCII blank pixels.
            else if(character>=8'd32 && character<8'd96 &&
                    character!=8'h40 && character!=8'h5b)
                font_code=character[5:0];
            else font_code=6'd32;
        end
    endfunction
    integer a;
    initial begin
        for(a=0;a<1024;a=a+1)
            rows[a]=glyph(a/16) >> (240-16*(a%16));
    end
    wire [9:0] addr_a={font_code(I_char_a),I_row_a};
    wire [9:0] addr_b={font_code(I_char_b),I_row_b};
    always @(posedge I_clk) begin
        O_row_a<=rows[addr_a];
        O_row_b<=rows[addr_b];
    end
endmodule
