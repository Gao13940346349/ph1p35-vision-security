// Ten short, distinct keypad feedback tones. The official board examples
// 13_music/a.adc and 9_MUX_buzz/AAA.adc identify the passive buzzer as C12.
// This implementation shares one divider and never creates an internal clock.
// All ten keys use the same debounced event path; reset is power/PLL only.
module keypad_tone #(
    parameter integer CLK_HZ = 100000000,
    parameter integer BEEP_CYCLES = CLK_HZ / 5
) (
    input wire clk,
    input wire rst_n,
    input wire [9:0] press_pulse, // K1 is bit 0; already debounced, one cycle.
    output reg buzzer,
    output wire active
);
    localparam integer MAX_HALF_PERIOD = (CLK_HZ + 523) / (2 * 523);
    localparam integer PW = MAX_HALF_PERIOD < 2 ? 1 : $clog2(MAX_HALF_PERIOD + 1);
    localparam integer DW = BEEP_CYCLES < 2 ? 1 : $clog2(BEEP_CYCLES + 1);

    reg [PW-1:0] half_period;
    reg [PW-1:0] phase_count;
    reg [DW-1:0] remaining;
    assign active = (remaining != 0);
    reg [3:0] selected_key;
    integer k;

    // Musical notes C5, D5, E5, F5, G5, A5, B5, C6, D6, E6.
    // All divisions are elaboration-time constants, not hardware dividers.
    function [PW-1:0] key_half_period;
        input [3:0] key;
        begin
            case (key)
                0: key_half_period = (CLK_HZ + 523) / (2 * 523);
                1: key_half_period = (CLK_HZ + 587) / (2 * 587);
                2: key_half_period = (CLK_HZ + 659) / (2 * 659);
                3: key_half_period = (CLK_HZ + 698) / (2 * 698);
                4: key_half_period = (CLK_HZ + 784) / (2 * 784);
                5: key_half_period = (CLK_HZ + 880) / (2 * 880);
                6: key_half_period = (CLK_HZ + 988) / (2 * 988);
                7: key_half_period = (CLK_HZ + 1047) / (2 * 1047);
                8: key_half_period = (CLK_HZ + 1175) / (2 * 1175);
                default: key_half_period = (CLK_HZ + 1319) / (2 * 1319);
            endcase
        end
    endfunction

    // A new press replaces the current tone and restarts its full duration.
    // Simultaneous presses use the highest-numbered key.
    always @* begin
        selected_key = 0;
        for (k = 0; k < 10; k = k + 1)
            if (press_pulse[k]) selected_key = k;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            half_period <= 0;
            phase_count <= 0;
            remaining <= 0;
            buzzer <= 1'b0;
        end else if (|press_pulse) begin
            half_period <= key_half_period(selected_key);
            phase_count <= key_half_period(selected_key) - 1'b1;
            remaining <= BEEP_CYCLES;
            buzzer <= 1'b1;
        end else if (remaining != 0) begin
            remaining <= remaining - 1'b1;
            if (remaining == 1) begin
                buzzer <= 1'b0;
                phase_count <= 0;
            end else if (phase_count == 0) begin
                phase_count <= half_period - 1'b1;
                buzzer <= ~buzzer;
            end else begin
                phase_count <= phase_count - 1'b1;
            end
        end else begin
            buzzer <= 1'b0;
        end
    end

    // synthesis translate_off
    initial begin
        if (CLK_HZ < 2638 || BEEP_CYCLES < 1)
            $fatal(1, "keypad_tone requires CLK_HZ >= 2638 and BEEP_CYCLES >= 1");
    end
    // synthesis translate_on
endmodule

// Common ten-key feedback and security-sound arbitration in one clock domain.
module keypad_tone_feedback #(
    parameter integer CLK_HZ = 100000000,
    parameter integer TONE_MS = 200,
    parameter integer SECURITY_SUPPORT = 0
) (
    input wire clk,
    input wire pll_locked,
    input wire [9:0] press_pulse,
    // Coherent, already synchronized application severity in this clock domain.
    input wire [2:0] security_level,
    output wire buzzer
);
    wire feedback_reset_n;
    local_reset_release u_feedback_reset(
        .clk(clk), .async_reset_n(pll_locked), .reset_n(feedback_reset_n));
    // Divide before multiplying to keep the millisecond conversion in range.
    wire key_buzzer, key_active;
    keypad_tone #(.CLK_HZ(CLK_HZ), .BEEP_CYCLES((CLK_HZ / 1000) * TONE_MS)) u_tone (
        .clk(clk), .rst_n(feedback_reset_n),
        .press_pulse(press_pulse), .buzzer(key_buzzer), .active(key_active));
    generate if (SECURITY_SUPPORT) begin: g_security
        security_buzzer #(.CLK_HZ(CLK_HZ)) u_security (
            .clk(clk), .rst_n(feedback_reset_n),
            .alarm_level(security_level),
            .key_start(|press_pulse),
            .key_active(key_active), .key_buzzer(key_buzzer), .buzzer(buzzer));
    end else begin: g_key_only
        assign buzzer=key_buzzer;
    end endgenerate
    // synthesis translate_off
    initial begin
        if ((CLK_HZ % 1000) != 0 || TONE_MS < 1)
            $fatal(1, "keypad_tone_feedback requires integer-kHz clock and positive duration");
    end
    // synthesis translate_on
endmodule

// A single final driver arbitrates security sounds and the existing key tone.
// Severity: 0 quiet; 1/2 blue/red warning; 3 blue intrusion; 4 red alarm.
// Warnings sound once on entry/change. A blue intrusion owns the buzzer even
// during its silent gaps; suppressed key sounds are never replayed afterwards.
// rst_n is the locally released power/PLL reset.
module security_buzzer #(
    parameter integer CLK_HZ=100000000,
    parameter integer WARNING_MS=100,
    parameter integer EVENT_ON_MS=200,
    parameter integer EVENT_OFF_MS=300
)(
    input wire clk,rst_n,
    input wire [2:0] alarm_level,
    input wire key_start,key_active,key_buzzer,
    output reg buzzer
);
    localparam integer MS_CYCLES=CLK_HZ/1000;
    localparam integer MW=MS_CYCLES<2 ? 1 : $clog2(MS_CYCLES);
    localparam integer WW=WARNING_MS<2 ? 1 : $clog2(WARNING_MS+1);
    localparam integer EVENT_MS=EVENT_ON_MS+EVENT_OFF_MS;
    localparam integer EW=EVENT_MS<2 ? 1 : $clog2(EVENT_MS);
    localparam integer MAX_HALF=(CLK_HZ+900)/(2*900);
    localparam integer PW=MAX_HALF<2 ? 1 : $clog2(MAX_HALF+1);
    reg [MW-1:0] ms_count;
    reg [WW-1:0] warning_remaining;
    reg [EW-1:0] event_phase;
    reg [2:0] previous_level;
    reg [PW-1:0] phase_count;
    reg wave,key_allowed;
    wire ms_tick=ms_count==MS_CYCLES-1;
    wire changed=alarm_level!=previous_level;
    wire warning_level=alarm_level==1 || alarm_level==2;
    wire warning_on=warning_level && (changed || warning_remaining!=0);
    wire security_owns=(alarm_level==3 || alarm_level==4 || warning_on);
    wire sound_on=alarm_level==4 || warning_on ||
                  (alarm_level==3 && (changed || event_phase<EVENT_ON_MS));
    reg [PW-1:0] half_period;
    always @* begin
        case(alarm_level)
            1:half_period=(CLK_HZ+900)/(2*900);
            2:half_period=(CLK_HZ+1500)/(2*1500);
            3:half_period=(CLK_HZ+1200)/(2*1200);
            default:half_period=(CLK_HZ+1800)/(2*1800);
        endcase
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            ms_count<=0; warning_remaining<=0; event_phase<=0;
            previous_level<=0; phase_count<=0; wave<=0; key_allowed<=1;
            buzzer<=0;
        end else begin
            // End the arbitration path at a local register instead of driving
            // it through the long route and HDIO output delay in this cycle.
            // This adds one clock of audio latency, with every divider/count
            // and all security/key suppression rules otherwise unchanged.
            buzzer<=security_owns ? (sound_on && wave) :
                    (key_allowed && key_active && key_buzzer);
            if(ms_tick) ms_count<=0; else ms_count<=ms_count+1'b1;
            previous_level<=alarm_level;
            if(changed) begin
                warning_remaining<=warning_level ? WARNING_MS : 0;
                event_phase<=0;
            end else if(ms_tick) begin
                if(warning_remaining!=0) warning_remaining<=warning_remaining-1'b1;
                if(alarm_level==3) begin
                    if(event_phase==EVENT_MS-1) event_phase<=0;
                    else event_phase<=event_phase+1'b1;
                end else event_phase<=0;
            end
            if(!sound_on) begin wave<=0;phase_count<=0;end
            else if(changed || phase_count==0) begin
                phase_count<=half_period-1'b1;
                wave<=changed ? 1'b1 : ~wave;
            end else phase_count<=phase_count-1'b1;
            if(security_owns) key_allowed<=0;
            else if(key_start) key_allowed<=1;
        end
    end
    // synthesis translate_off
    initial begin
        if(CLK_HZ<4000 || CLK_HZ%1000!=0 || WARNING_MS<1 || EVENT_ON_MS<1 || EVENT_OFF_MS<1)
            $fatal(1,"security_buzzer requires integer-kHz clock >=4kHz and positive durations");
    end
    // synthesis translate_on
endmodule
