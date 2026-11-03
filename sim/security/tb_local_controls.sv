`timescale 1ns/1ps
module tb_local_controls;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0;
    reg [9:0] key=0;
    reg [1:0] legacy_ag=0;
    reg [6:0] current_again=0;
    wire awb,ae,cal,detect,application;
    wire [2:0] mode;
    wire [11:0] manual_exposure;
    wire [6:0] manual_again,manual_dgain;
    wire manual_exposure_en,manual_again_en,manual_dgain_en,param_toggle,restore_toggle;
    local_control_state dut(.clk(clk),.rst_n(rst_n),
        .btn_mode(key[0]),.btn_detect(key[1]),.btn_application(key[2]),
        .btn_dg_up(key[3]),.btn_awb(key[9]),.btn_cal(key[5]),.btn_ae(key[6]),
        .btn_dg_down(key[8]),.btn_defaults(1'b0),
        .btn_ag_up(legacy_ag[0]),.btn_ag_down(legacy_ag[1]),.current_again(current_again),
        .awb(awb),.ae(ae),.cal(cal),.detect(detect),.application(application),.mode(mode),
        .manual_exposure(manual_exposure),.manual_again(manual_again),.manual_dgain(manual_dgain),
        .manual_exposure_en(manual_exposure_en),.manual_again_en(manual_again_en),
        .manual_dgain_en(manual_dgain_en),.param_toggle(param_toggle),.restore_toggle(restore_toggle));
    wire [38:0] configuration={awb,ae,cal,detect,application,mode,
        manual_exposure,manual_again,manual_dgain,
        manual_exposure_en,manual_again_en,manual_dgain_en,param_toggle,restore_toggle};
    always @(posedge clk) begin
        #1;
        if(rst_n && application && !detect)$fatal(1,"application lost the shared vision enable");
        if(rst_n && cal && detect)$fatal(1,"details overlaps a vision page");
        if(rst_n && (manual_again_en || manual_again!=32))$fatal(1,"analogue gain left automatic ownership");
        if(rst_n && restore_toggle)$fatal(1,"a physical key requested the removed global restore");
    end
    task reset;
        @(negedge clk);rst_n=0;key=0;legacy_ag=0;
        repeat(2)@(negedge clk);rst_n=1;@(negedge clk);
    endtask
    task press(input [9:0] mask);
        @(negedge clk);key=mask;@(negedge clk);key=0;
    endtask
    task check(input bit ok,input string message);
        if(!ok)$fatal(1,"%s keys=%b state awb=%b ae=%b details=%b detect=%b app=%b mode=%d",
            message,key,awb,ae,cal,detect,application,mode);
    endtask
    reg [9:0] chord;
    reg [38:0] before_configuration;
    integer expected_page,page_checks=0;
    initial begin
        // Exhaust every page-key chord with and without independent K10 AWB.
        // K5 NEXT and K8 ACK are business events, never camera configuration.
        for(integer page=0;page<4;page++)begin
            for(integer mask=0;mask<16;mask++)begin
              for(integer white_balance=0;white_balance<2;white_balance++)begin
                reset();
                case(page)
                    1:press(10'b0000000010);
                    2:press(10'b0000000100);
                    3:press(10'b0000100000);
                endcase
                chord=10'b0010010000;
                chord[9]=white_balance;
                chord[0]=mask[0];chord[1]=mask[1];chord[2]=mask[2];chord[5]=mask[3];
                press(chord);
                // Documented winner: K1 > K3 > K2 > K6. Repeating the winning
                // page key exits that page; pressing a different one enters it.
                if(mask[0])expected_page=0;
                else if(mask[2])expected_page=page==2 ? 0 : 2;
                else if(mask[1])expected_page=page==1 ? 0 : 1;
                else if(mask[3])expected_page=page==3 ? 0 : 3;
                else expected_page=page;
                check({cal,application,detect}==
                    (expected_page==3 ? 3'b100 : expected_page==2 ? 3'b011 :
                     expected_page==1 ? 3'b001 : 3'b000),"page chord precedence");
                check(mode==(mask[0] ? 1 : 0),"effect mode changes only with K1");
                check(awb==!white_balance,"K10 AWB is independent of previous/next page");
                page_checks++;
              end
            end
        end
        reset();
        repeat(8)press(10'b0000000001);check(mode==0,"K1 eight-mode wrap");
        press(10'b0000000010);press(10'b0000000010);
        check(!detect && !application,"K2 toggles tracking on/off");
        press(10'b0000000100);press(10'b0000000100);
        check(!detect && !application,"K3 toggles application on/off");
        press(10'b0000001000);check(manual_dgain==16 && manual_dgain_en,"K4 digital +0.5");
        press(10'b0100000000);check(manual_dgain==0,"K9 digital -0.5");
        before_configuration=configuration;
        press(10'b0100001000);check(configuration==before_configuration,"opposing digital gain cancels");
        press(10'b0001000000);check(!ae && manual_exposure_en,"K7 AE disable");
        press(10'b0001000000);check(ae && !manual_exposure_en,"K7 AE enable");
        // The two obsolete analogue-step inputs and K5/K8 cannot change ANY state,
        // including the CDC event toggle, irrespective of the reported AE gain.
        before_configuration=configuration;
        for(integer gain=0;gain<128;gain++)begin
            current_again=gain;
            for(integer combination=0;combination<4;combination++)begin
                @(negedge clk);legacy_ag=combination;key=10'b0010010000;
                @(negedge clk);legacy_ag=0;key=0;
                check(configuration==before_configuration,"K5/K8/legacy analogue keys cannot write local state");
            end
        end
        // K10 must preserve the selected page, disabled AE, digital gain and
        // both CDC events, including when pressed repeatedly in application.
        press(10'b0000000100);press(10'b0000001000);press(10'b0001000000);
        before_configuration=configuration;
        press(10'b1000000000);
        check(configuration[37:0]==before_configuration[37:0] && awb!=before_configuration[38],
            "K10 only toggles AWB, preserving all other settings");
        press(10'b1000000000);
        check(configuration==before_configuration,"second K10 restores AWB only");
        press(10'b1111111111);
        check(!awb && ae && !cal && !detect && !application && mode==1 &&
            manual_exposure==1990 && manual_again==32 && manual_dgain==16 &&
            !manual_exposure_en && !manual_again_en && manual_dgain_en,
            "all-key chord uses page priority, independent AWB/AE, and opposed digital cancel");
        reset();check(restore_toggle==0 && param_toggle==0,"asynchronous reset clears CDC events");
        $display("PASS local_controls: %0d page/AWB chords, independent K10 AWB preserving configuration, K2/K3 exclusivity, K5/K8 and 512 legacy gain combinations configuration-inert, no physical restore",page_checks);
        $finish;
    end
    initial begin #2000000;$fatal(1,"control timeout");end
endmodule
