`timescale 1ns/1ps
module tb_keypad10;
    reg clk=0;always #5 clk=~clk;
    reg pll_locked=0;
    wire rst_n;
    // Both a normal key and K10 are held at power-up. Neither is a reset.
    reg [9:0] keys=10'h1fe;
    wire [9:0] press;
    wire application;
    wire awb,ae,cal,detect,ag_en,dg_en,exp_en,pt,rt;
    wire [2:0] mode;
    wire [6:0] ag,dg;
    wire [11:0] exposure;
    integer events[0:9];
    integer before_count;
    reg before_chord_awb;
    reg [37:0] before_configuration;
    wire [37:0] configuration={ae,cal,detect,application,mode,exposure,ag,dg,
        exp_en,ag_en,dg_en,pt,rt};
    local_reset_release reset_release(.clk(clk),.async_reset_n(pll_locked),.reset_n(rst_n));
    keypad10 #(.TICK_CYCLES(4)) pad(.clk(clk),.rst_n(rst_n),.keys_n(keys),.press(press));
    local_control_state state(.clk(clk),.rst_n(rst_n),.btn_awb(press[9]),
        .btn_mode(press[0]),.btn_detect(press[1]),.btn_application(press[2]),
        .btn_ag_up(1'b0),.btn_ag_down(1'b0),.btn_dg_up(press[3]),
        .btn_cal(press[5]),.btn_ae(press[6]),.btn_dg_down(press[8]),
        .btn_defaults(1'b0),.current_again(7'd46),
        .awb(awb),.ae(ae),.cal(cal),.detect(detect),.application(application),
        .mode(mode),.manual_exposure(exposure),.manual_again(ag),.manual_dgain(dg),
        .manual_exposure_en(exp_en),.manual_again_en(ag_en),.manual_dgain_en(dg_en),
        .param_toggle(pt),.restore_toggle(rt));
    always @(posedge clk) if(rst_n) begin
        for(integer k=0;k<10;k++)if(press[k])events[k]++;
        if(rt)$fatal(1,"physical keypad requested removed global restore");
    end
    task key(input integer k);
        @(negedge clk);keys[k-1]=0;repeat(48)@(negedge clk);
        keys[k-1]=1;repeat(48)@(negedge clk);
    endtask
    task awb_only;
        reg before_awb;
        begin
            before_configuration=configuration;before_awb=awb;
            key(10);
            if(awb===before_awb || configuration!==before_configuration || !rst_n)
                $fatal(1,"K10 must toggle AWB only, without reset or configuration loss");
        end
    endtask
    task business_only(input integer k);
        reg before_awb;
        begin
            before_configuration=configuration;before_awb=awb;
            before_count=events[k-1];key(k);
            if(configuration!==before_configuration || awb!==before_awb || events[k-1]!=before_count+1)
                $fatal(1,"K%0d business event changed local camera/page configuration",k);
        end
    endtask
    initial begin
        for(integer k=0;k<10;k++)events[k]=0;
        repeat(5)@(negedge clk);pll_locked=1;repeat(100)@(negedge clk);
        if(!rst_n || !awb || mode!=0 || events[0]!=0 || events[9]!=0)
            $fatal(1,"power-up held K10 reset the system or generated an event");
        keys='1;repeat(48)@(negedge clk);
        // Contact bounce is shorter than eight shared ticks.
        repeat(5)begin keys[9]=0;repeat(2)@(negedge clk);keys[9]=1;repeat(2)@(negedge clk);end
        repeat(48)@(negedge clk);
        if(!awb || events[9]!=0)$fatal(1,"K10 bounce toggled AWB");
        keys[9]=0;repeat(200)@(negedge clk);
        if(awb || events[9]!=1 || !rst_n)$fatal(1,"held K10 repeated or asserted reset");
        keys[9]=1;repeat(48)@(negedge clk);awb_only();
        key(1);repeat(7)key(1);if(mode!=0)$fatal(1,"K1 eight-mode wrap");
        key(4);key(7);
        if(dg!=16 || !dg_en || ae || !exp_en)$fatal(1,"parameter setup failed");
        // Every page supports independent AWB; neither NEXT nor ACK changes
        // camera settings. Top-level wiring and tracker gating are checked
        // separately against the actual production RTL.
        business_only(5);business_only(8);awb_only();
        key(2);if(!detect || application)$fatal(1,"K2 detection selection");
        business_only(5);business_only(8);awb_only();
        key(3);if(!detect || !application)$fatal(1,"K3 application selection");
        business_only(5);business_only(8);awb_only();
        key(6);if(!cal || detect || application)$fatal(1,"K6 details selection");
        business_only(5);business_only(8);awb_only();
        if(dg!=16 || !dg_en || ae || !exp_en || ag!=32 || ag_en)
            $fatal(1,"AWB/page switches lost camera parameter ownership");
        // Simultaneous application entry and AWB must do both operations.
        before_chord_awb=awb;
        keys[2]=0;keys[9]=0;repeat(48)@(negedge clk);keys='1;repeat(48)@(negedge clk);
        if(!application || !detect || awb===before_chord_awb || cal || dg!=16 || ae)
            $fatal(1,"K3/K10 chord must enter application and toggle AWB independently");
        repeat(35)key(4);if(dg!=127 || ag!=32 || ag_en)$fatal(1,"digital upper clamp");
        repeat(35)key(9);if(dg!=0 || ag!=32 || ag_en)$fatal(1,"digital lower clamp");
        // Only genuine PLL loss resets all state, with K10 held through relock.
        keys[9]=0;pll_locked=0;#1;
        if(rst_n!==0)$fatal(1,"PLL loss did not assert reset asynchronously");
        repeat(4)@(negedge clk);pll_locked=1;repeat(100)@(negedge clk);
        if(!rst_n || !awb || !ae || detect || application || cal || mode!=0 || dg_en || rt)
            $fatal(1,"PLL relock defaults or held-key startup suppression failed");
        keys[9]=1;repeat(48)@(negedge clk);awb_only();
        $display("PASS keypad10: K10 independent AWB, power-up/relock held, bounce/hold/repress, all pages, K5/K8 business events, K3/K10 chord, parameters preserved, digital clamps, PLL reset");
        $finish;
    end
    initial begin #2000000;$fatal(1,"watchdog");end
endmodule
