`timescale 1ns/1ps
module tb_security_buzzer;
    localparam integer HZ=100000, MS=HZ/1000;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0;
    reg [2:0] level=0;
    reg key_start=0,key_active=0,key_wave=0;
    wire buzzer;
    security_buzzer #(.CLK_HZ(HZ),.WARNING_MS(5),.EVENT_ON_MS(4),.EVENT_OFF_MS(6)) dut(
        .clk(clk),.rst_n(rst_n),.alarm_level(level),.key_start(key_start),
        .key_active(key_active),.key_buzzer(key_wave),.buzzer(buzzer));
    integer checks=0;
    task automatic step(input integer n);
        repeat(n) begin @(posedge clk);#1;checks=checks+1;end
    endtask
    task automatic expect_silent(input integer n);
        repeat(n) begin
            @(posedge clk);#1;checks=checks+1;
            if(buzzer!==0) $fatal(1,"Unexpected sound level %0d",level);
        end
    endtask
    task automatic set_level(input [2:0] v);
        @(negedge clk);level=v;
        @(posedge clk);#1;
        // Final arbitration is registered: observe its one-clock-delayed wave.
        @(posedge clk);#1;
    endtask
    task automatic press;
        @(negedge clk);key_active=1;key_wave=1;key_start=1;
        @(posedge clk);#1;
        @(negedge clk);key_start=0;
        @(posedge clk);#1;
    endtask
    task automatic tone_check(input integer freq);
        integer half_period;
        reg first;
        begin
            half_period=(HZ+freq)/(2*freq);
            first=buzzer;
            if(first!==1) $fatal(1,"Sound did not start at level %0d",level);
            repeat(half_period-1) begin
                @(posedge clk);#1;
                if(buzzer!==first) $fatal(1,"Pitch toggled too early level %0d",level);
            end
            @(posedge clk);#1;
            if(buzzer!==!first) $fatal(1,"Pitch half-period mismatch level %0d",level);
            checks=checks+half_period;
        end
    endtask

    // Full production helper: K10 AWB has exactly ordinary key priority.
    // It cannot reset or acknowledge a security alarm.
    reg pll_locked=0;
    reg [9:0] key_pulse=0;
    reg [2:0] feedback_level=0;
    wire feedback_buzzer;
    keypad_tone_feedback #(.CLK_HZ(HZ),.TONE_MS(20),.SECURITY_SUPPORT(1)) feedback(
        .clk(clk),.pll_locked(pll_locked),
        .press_pulse(key_pulse),.security_level(feedback_level),.buzzer(feedback_buzzer));
    integer i,transitions;
    reg old;
    initial begin
        step(5);@(negedge clk);rst_n=1;
        expect_silent(10);
        press();
        if(buzzer!==1) $fatal(1,"Idle key did not pass");
        @(negedge clk);key_wave=0;step(1);
        if(buzzer!==0) $fatal(1,"Key waveform was not passed");
        @(negedge clk);key_wave=1;
        set_level(1);tone_check(900);
        step(6*MS);
        expect_silent(12*MS); // no repeated warning; old key cannot resume
        press();step(1);
        if(buzzer!==1) $fatal(1,"New key after finished warning was blocked");
        set_level(2);tone_check(1500);
        step(6*MS);expect_silent(12*MS);
        set_level(0);expect_silent(10);
        set_level(1);tone_check(900); // leaving/reentering rearms warning
        set_level(4);tone_check(1800); // alarm preempts warning immediately
        press(); // a key during alarm is consumed rather than queued
        step(MS);
        set_level(0);expect_silent(5*MS);
        press();step(1);
        if(buzzer!==1) $fatal(1,"New key after alarm was blocked");
        set_level(3);tone_check(1200);
        // Stable intermittent alarm: its silence must also suppress keys.
        wait(dut.event_phase==5);#1;
        expect_silent(2*MS);
        press();expect_silent(MS);
        wait(dut.event_phase==0);step(2);
        transitions=0;old=buzzer;
        repeat(2*MS) begin
            @(posedge clk);#1;
            if(buzzer!=old) transitions=transitions+1;
            old=buzzer;
        end
        if(transitions<3) $fatal(1,"Intermittent alarm did not resume");
        set_level(0);expect_silent(MS);
        @(negedge clk);rst_n=0;#1;
        if(buzzer!==0) $fatal(1,"PLL reset did not silence output");

        @(negedge clk);pll_locked=1;
        step(10);
        if(feedback_buzzer!==0) $fatal(1,"PLL startup created a false key sound");
        @(negedge clk);feedback_level=4;
        step(100);
        @(negedge clk);key_pulse=10'b1000000000; // K10 AWB while alarm sounding
        step(2);
        if(!feedback.feedback_reset_n || feedback.g_security.u_security.key_allowed!==0 ||
            feedback.g_security.u_security.previous_level!==4)
            $fatal(1,"K10 reset or overrode security alarm ownership");
        @(negedge clk);key_pulse=0;
        transitions=0;old=feedback_buzzer;
        repeat(15*MS) begin
            @(posedge clk);#1;
            if(feedback_buzzer!=old) transitions=transitions+1;
            old=feedback_buzzer;
        end
        if(transitions<30) $fatal(1,"Red alarm stopped after K10 AWB");
        step(10*MS);
        if(feedback.g_security.u_security.previous_level!==4)
            $fatal(1,"K10 AWB unexpectedly cleared the application alarm");
        // K8 creates an ordinary key tone; application ACK clears severity
        // separately after its CDC/controller delay. Suppressed tones stay off.
        @(negedge clk);key_pulse=10'b0010000000;
        step(2);@(negedge clk);key_pulse=0;step(10);
        @(negedge clk);feedback_level=0;
        step(15);
        if(feedback_buzzer!==0) $fatal(1,"ACK replayed a stale suppressed key tone");
        @(negedge clk);key_pulse=1;
        @(posedge clk);#1;
        @(posedge clk);#1;
        if(feedback_buzzer!==1) $fatal(1,"Normal key after K10 was blocked");
        @(negedge clk);key_pulse=0;pll_locked=0;#1;
        if(feedback_buzzer!==0) $fatal(1,"PLL loss did not cancel output");
        $display("PASS security buzzer: %0d checks, warning pitches/one shot, continuous/intermittent priority, no replay, K10 AWB cannot clear/preempt alarm, K8 ACK severity release, startup and PLL reset",checks);
        $finish;
    end
    initial begin #2000000;$fatal(1,"Timeout");end
endmodule
