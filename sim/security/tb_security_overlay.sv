`timescale 1ns/1ps
module tb_security_overlay;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,de=0,user=0,enable=1;
    reg [11:0] x=0,y=0;
    reg [2:0] app_state=1,severity=0;
    reg [15:0] high_count_bcd=16'h0123,suspicious_count_bcd=16'h0045,event_source_id=16'hbeef;
    wire hit;wire [23:0] colour;
    security_overlay dut(.*);
    task automatic frame_start;
        @(negedge clk);user=1;de=0;
        @(negedge clk);user=0;
        repeat(3) @(negedge clk);
    endtask
    task automatic sample(input integer px,py,input bit expected_hit,input [23:0] expected_colour);
        @(negedge clk);x=px;y=py;de=1;
        repeat(2) @(posedge clk);#1;
        if(hit!==expected_hit || (expected_hit && colour!==expected_colour))
            $fatal(1,"security pixel (%0d,%0d) hit=%b/%b colour=%h/%h",px,py,hit,expected_hit,colour,expected_colour);
    endtask
    function automatic [23:0] scene(input integer px,py);
        integer dx,dy;
        begin
            scene=24'he9ecef;
            // Synthetic source only; security OSD pixels come from real RTL.
            dx=px-950;dy=py-390;
            if(dx*dx+dy*dy<10000)scene=24'he95843;
            if(py>315 && py<515 && px>180 && px<380)scene=24'h69bd87;
            if(py>305 && py<505 && px>640-(py-305)/2 && px<640+(py-305)/2)scene=24'h4498d7;
        end
    endfunction
    integer i,px,py,fh,total_hits,body_hits;
    reg previous_hit;
    reg [23:0] previous_colour,bg;
    reg [11:0] previous_x,previous_y;
    initial begin
        repeat(4) @(negedge clk);rst_n=1;
        frame_start();
        if(dut.panel_char(2,0)!="C" || dut.panel_char(2,4)!="P" ||
           dut.panel_char(2,12)!="S" || dut.panel_char(2,16)!="E" ||
           dut.panel_char(2,23)!="T" || dut.panel_char(2,27)!="V" ||
           dut.panel_char(2,37)!="K" || dut.panel_char(2,38)!="8" ||
           dut.panel_char(2,40)!="A" || dut.panel_char(3,6)!="A" ||
           dut.panel_char(3,17)!="V" || dut.panel_char(3,30)!="R")
            $fatal(1,"object-type/grade legend and K8 ACK hint must match application semantics");
        if(dut.status_char(0)!="M" || dut.panel_char(0,37)!="0" || dut.panel_char(0,38)!="1" ||
           dut.panel_char(0,39)!="2" || dut.panel_char(0,40)!="3" || dut.panel_char(0,51)!="4" ||
           dut.panel_char(0,52)!="5" || dut.panel_char(1,46)!="B" || dut.panel_char(1,49)!="F" ||
           dut.zone_char(0,0)!="S" || dut.zone_char(1,0)!="W" || dut.zone_char(2,0)!="R")
            $fatal(1,"frame text/counters/source incorrect");
        sample(0,0,0,0);sample(100,60,0,0); // Entire existing logo remains untouched.
        sample(176,8,1,24'h50e890); // State stripe.
        sample(180,9,1,24'h102030);
        sample(188,17,1,24'heaf2ff); // Existing font S in SECURITY.
        sample(346,17,1,24'h50e890); // Existing font M in MONITOR.
        sample(1,160,1,24'h50e890);sample(511,160,1,24'h50e890);
        sample(512,160,1,24'h70b8ff);sample(767,160,1,24'h70b8ff);
        sample(768,160,1,24'hff6060);sample(1278,160,1,24'hff6060);
        sample(512,168,0,0); // Dashed gap.
        sample(640,360,0,0);sample(1279,719,0,0);
        sample(510,160,0,0); // Border stays narrow.
        sample(220,113,1,24'h102030);sample(200,120,0,0);
        sample(228,117,1,24'h50e890); // SAFE label uses zone color.
        sample(224,116,1,24'h102030); // Font margin, not a filled zone.
        // Status and numbers cannot tear when incoming mailbox changes mid-frame.
        app_state=2;severity=4;high_count_bcd=16'h0003;event_source_id=16'h00a5;
        if(dut.status_char(0)!="M" || dut.panel_char(0,38)!="1")$fatal(1,"mid-frame metadata tore");
        sample(768,160,1,24'hff6060);
        frame_start();
        if(dut.status_char(0)!="H" || dut.panel_char(0,40)!="3" || dut.panel_char(1,0)!="E" ||
           dut.panel_char(1,16)!="M" || dut.panel_char(1,46)!="0" || dut.panel_char(1,48)!="A")
            $fatal(1,"alarm state/count/paused marker wrong");
        sample(768,160,1,24'h738498); // Moving-camera zones are reference only.
        while(!dut.blink_frame[4]) frame_start();
        sample(176,8,1,24'h102030); // Only narrow state stripe flashes off.
        while(dut.blink_frame[4]) frame_start();
        sample(176,8,1,24'hff6060);
        app_state=3;frame_start();
        if(dut.status_char(0)!="A" || dut.panel_char(1,16)!="R" || dut.panel_char(0,40)!="3")
            $fatal(1,"ACK keeps count and explicitly waits clear");
        enable=0;sample(176,8,0,0);sample(512,160,0,0);sample(220,113,0,0);
        enable=1;
        // Alternating active/inactive pixels verify two-stage pipeline, including DE tail.
        @(negedge clk);de=0;repeat(3)@(posedge clk);
        previous_hit=0;
        for(i=0;i<12;i=i+1)begin
            @(negedge clk);x=i[0]?640:512;y=160;de=1;
            @(posedge clk);#1;
            if(hit!==previous_hit || (previous_hit && colour!==24'h70b8ff))
                $fatal(1,"stream alignment %0d",i);
            previous_hit=!i[0];
        end
        @(negedge clk);de=0;
        @(posedge clk);#1;if(hit!==previous_hit)$fatal(1,"stream tail missing");
        @(posedge clk);#1;if(hit!==0)$fatal(1,"DE tail incorrect");
        // Generate a full 720p synthetic-input preview using the actual OSD output.
        app_state=2;severity=4;frame_start();
        fh=$fopen("security_overlay_preview.ppm","w");
        if(fh==0)$fatal(1,"preview open failed");
        $fwrite(fh,"P3\n1280 720\n255\n");
        total_hits=0;body_hits=0;previous_x=0;previous_y=0;
        @(negedge clk);de=0;repeat(3)@(posedge clk);
        for(py=0;py<720;py=py+1)begin
            for(px=0;px<1280;px=px+1)begin
                @(negedge clk);de=1;x=px;y=py;
                @(posedge clk);#1;
                if(px!=0 || py!=0)begin
                    if($isunknown({hit,colour}))$fatal(1,"unknown pixel at %0d,%0d",previous_x,previous_y);
                    if(hit)begin
                        total_hits=total_hits+1;
                        if(previous_y>=144)body_hits=body_hits+1;
                        bg=colour;
                    end else bg=scene(previous_x,previous_y);
                    if(previous_x<176 && previous_y<104 && hit)$fatal(1,"logo overwritten");
                    $fwrite(fh,"%0d %0d %0d\n",bg[23:16],bg[15:8],bg[7:0]);
                end
                previous_x=px;previous_y=py;
            end
        end
        @(negedge clk);de=0;
        @(posedge clk);#1;
        bg=hit?colour:scene(previous_x,previous_y);
        $fwrite(fh,"%0d %0d %0d\n",bg[23:16],bg[15:8],bg[7:0]);
        $fclose(fh);
        if(total_hits<90000 || total_hits>100000 || body_hits<2500 || body_hits>4000)
            $fatal(1,"full-frame coverage wrong total=%0d body=%0d",total_hits,body_hits);
        $display("PASS security_overlay: 720p full frame, two-stage pixel stream, logo transparency, narrow dashed zones, mode/BCD/hex text, frame atomicity, paused monitor and ACK visibility; preview is synthetic input + real RTL OSD");
        $finish;
    end
    initial begin #30000000;$fatal(1,"security overlay timeout");end
endmodule
