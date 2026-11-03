`timescale 1ns/1ps
module tb_security_application #(parameter SHARED=0);
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,enable=0,ack=0,strobe=0,bad=0;
    reg [15:0] id=0,capture=0;
    reg [422:0] boxes=0;
    wire [2:0] state,severity;
    wire [46:0] seed;
    wire tracking,publish;
    wire [15:0] red,blue,source;
    security_application #(.SHARED_BOXES(SHARED),.CAMERA_CLK_HZ(1000),.SETTLE_MS(3)) dut(
        .clk(clk),.rst_n(rst_n),.enable(enable),.ack(ack),
        .result_strobe(strobe),.result_bad(bad),.result_id(id),.capture_id(capture),.boxes(boxes),
        .app_state(state),.severity(severity),.event_seed(seed),.track_enable(tracking),
        .red_count_bcd(red),.blue_count_bcd(blue),.event_source_id(source),.status_publish(publish));
    function automatic [46:0] box(input integer c,s,x,y);
        box={2'(c),2'(s),1'b1,11'(x-30),11'(x+30),10'(y-30),10'(y+30)};
    endfunction
    task automatic check(input bit ok,input string msg);
        if(!ok) $fatal(1,"%s state=%d severity=%d red=%h blue=%h",msg,state,severity,red,blue);
    endtask
    task automatic start;
        @(negedge clk);enable=0;rst_n=0;strobe=0;bad=0;ack=0;boxes=0;id=0;capture=0;
        repeat(4) @(negedge clk);rst_n=1;enable=1;
        repeat(10) @(negedge clk);check(state==1,"enter MONITOR after mechanical settle");
    endtask
    task automatic frame(input [422:0] value,input bit invalid=0,input bit duplicate=0);
        @(negedge clk);boxes=value;bad=invalid;
        if(!duplicate) begin id=id+1'b1;capture=id;end
        strobe=1;@(negedge clk);strobe=0;
        repeat(40) @(negedge clk);bad=0;
    endtask
    task automatic acknowledge;
        @(negedge clk);ack=1;@(negedge clk);ack=0;
        repeat(10) @(negedge clk);
        check(state==3 && !tracking,"ACK returns and settles while retaining event");
    endtask
    reg [422:0] set;
    integer n,next_n,c,s,z,expected_level,category_slot;reg [15:0] value_bcd,next_bcd;
    initial begin
        start();
        for(n=0;n<10000;n=n+1) begin
            next_n=n==9999 ? 9999 : n+1;
            value_bcd=((n/1000)<<12)|(((n/100)%10)<<8)|(((n/10)%10)<<4)|(n%10);
            next_bcd=((next_n/1000)<<12)|(((next_n/100)%10)<<8)|(((next_n/10)%10)<<4)|(next_n%10);
            check(dut.increment_bcd(value_bcd)==next_bcd,"decimal event counter carry and saturation");
        end
        // Every object type is authorized when green, even in restriction.
        set=0;set[0+:47]=box(2,2,1000,300);set[47+:47]=box(2,0,900,500);
        set[94+:47]=box(2,1,1000,500);
        repeat(5) frame(set);check(severity==0 && red==0 && blue==0,"all authorized object types remain clear");
        // Three NEW coherent source results, not repeated DDR display or duplicate IDs.
        set=0;set[8*47+:47]=box(1,2,1000,300);
        frame(set);repeat(4) frame(set,0,1);check(!tracking,"duplicates cannot confirm event");
        frame(set);check(!tracking,"two source results insufficient");
        frame(set);check(state==2 && red==1 && blue==0 && seed==set[8*47+:47],"slot8 event locks correct box");
        check(source==id,"incident remembers confirming source ID");
        repeat(8) frame(set);check(red==1,"persistent and moving-camera frames cannot recount");
        acknowledge();
        repeat(8) frame(set);check(state==3 && red==1,"ACK remains silent waiting for same threat to clear");
        repeat(5) frame(0);check(state==3,"clear debounce requires six results");
        frame(0,1);frame(0);check(state==3,"bad frame breaks clear evidence, does not clear");
        repeat(5) frame(0);check(state==1 && red==1,"clear returns to monitor and preserves count");
        repeat(3) frame(set);check(red==2 && state==2,"reentry creates next event");
        @(negedge clk);enable=0;repeat(5) @(negedge clk);check(red==2 && !tracking,"mode exit retains counters");
        enable=1;repeat(10) @(negedge clk);check(state==1 && red==2,"new monitoring session");
        // Priority across all nine detections, unrelated to normal selected target.
        start();set=0;set[0+:47]=box(3,0,900,400);set[47+:47]=box(1,0,1000,400);
        repeat(3) frame(set);check(red==1 && blue==0 && seed[46:45]==1,"red intrusion wins global priority");
        // Blue restricted target event and category-limited clear.
        start();set=0;set[47+:47]=box(3,0,900,400);
        repeat(3) frame(set);check(blue==1 && red==0 && severity==3,"blue vehicle intrusion");
        acknowledge();repeat(6) frame(0);check(state==1,"blue incident rearmed");
        // Warning hysteresis and once-per-episode severity publication.
        start();set=0;set[0+:47]=box(3,2,600,350);
        repeat(3) frame(set);check(state==1 && severity==1 && blue==0,"blue warning does not steer/count");
        set[0+:47]=box(3,2,508,350);repeat(4) frame(set);check(severity==1,"warning boundary hysteresis");
        set[0+:47]=box(3,2,490,350);repeat(6) frame(set);check(severity==0,"warning clears outside hysteresis");
        set[0+:47]=box(1,0,600,350);repeat(3) frame(set);check(severity==2 && red==0,"red triangle warning");
        acknowledge();repeat(8) frame(set);check(state==3,"warning ACK cannot immediately sound again");
        repeat(6) frame(0);check(state==1,"warning episode cleared");
        // ACK clear scans all candidates, even if a different category dominates.
        set=0;set[0+:47]=box(3,2,600,350);repeat(3) frame(set);
        acknowledge();set[47+:47]=box(1,0,650,350);repeat(8) frame(set);
        check(state==3,"warning ACK cannot be hidden by higher-priority other category");
        set[0+:47]=0;repeat(6) frame(set);check(state==1,"warning ACK clears after original category exits");
        // Similar category in alternating distant locations cannot pool confirmations.
        start();
        repeat(3) begin
            set=0;set[0+:47]=box(1,2,820,300);frame(set);
            set[0+:47]=box(1,2,1150,300);frame(set);
        end
        check(red==0 && !tracking,"different distant objects cannot pool confirmation");
        repeat(2) frame(set);check(red==1,"same associated target can confirm");
        // Explicitly bad results break entry evidence.
        start();set=0;set[0+:47]=box(1,2,900,300);frame(set);frame(set,1);frame(set);frame(set);
        check(!tracking,"bad frame breaks entry confirmation");frame(set);check(tracking,"reconfirmed after bad frame");
        // Reset clears all counters, unlike ACK and mode exit.
        @(negedge clk);rst_n=0;repeat(3) @(negedge clk);check(red==0 && blue==0,"system reset clears counters");
        // Correct modulo-16-bit source ordering through wrap.
        rst_n=1;enable=0;capture=16'hfffe;id=16'hfffe;repeat(4) @(negedge clk);enable=1;
        repeat(10) @(negedge clk);frame(set);frame(set);frame(set);check(red==1,"source ID wraps without losing event");
        if(SHARED) begin
            start();set=0;set[8*47+:47]=box(1,2,950,350);
            @(negedge clk);boxes=set;id=id+1'b1;capture=id;strobe=1;
            @(negedge clk);strobe=0;repeat(5)@(negedge clk);
            // Simulate a producer replacing the shared snapshot mid-scan.
            boxes=0;id=id+1'b1;capture=id;strobe=1;
            @(negedge clk);strobe=0;repeat(40)@(negedge clk);
            check(red==0 && !tracking && dut.confirm_count==0,"overwritten shared scan cannot become evidence");
            repeat(3)frame(set);check(red==1 && tracking,"fresh frames recover after overwritten scan");
        end
        // Orthogonal colour/shape semantics: nine categories x three zones.
        // Rotate through all slots so the new equipment category is not tied
        // to the usual detector slot order or to a normal selected target.
        for(c=1;c<=3;c=c+1)for(s=0;s<=2;s=s+1)for(z=0;z<=2;z=z+1)begin
            start();set=0;category_slot=(c*3+s+z)%9;
            set[category_slot*47+:47]=box(c,s,z==0?300:z==1?640:1000,350);
            expected_level=(c==2 || z==0)?0:z==1?(c==1?2:1):(c==1?4:3);
            repeat(3)frame(set);
            check(severity==expected_level,"colour grade controls policy independently of shape");
            check(tracking==(expected_level>=3),"only confirmed restricted-zone entry steers");
            check(red==(expected_level==4) && blue==(expected_level==3),"only confirmed incident increments matching grade counter");
            if(expected_level!=0)check(seed==set[category_slot*47+:47] && source==id,
                "event metadata retains exact colour, object type, geometry and source ID");
            if(expected_level!=0)begin
                acknowledge();repeat(7)frame(set);
                check(state==3,"ACK suppresses the original person/equipment/vehicle category");
                repeat(6)frame(0);
                check(state==1 && severity==0,"every warned/incident category clears and rearms");
            end
        end
        // Undefined detector categories are never silently interpreted as an
        // authorized or restricted object, regardless of a valid rectangle.
        start();set=0;set[0+:47]=box(0,2,1000,350);set[47+:47]=box(1,3,1000,350);
        repeat(5)frame(set);check(severity==0 && !tracking && red==0 && blue==0,"undefined colour and shape are rejected");
        // ACK must not confuse different object types with the same grade.
        start();set=0;set[0+:47]=box(1,1,1000,350);repeat(3)frame(set);
        check(state==2 && seed[44:43]==1,"restricted equipment can trigger and seed tracking");
        acknowledge();set=0;set[0+:47]=box(1,2,1000,350);repeat(6)frame(set);
        check(state==1 && red==1,"person does not block clearing acknowledged equipment");
        repeat(3)frame(set);check(state==2 && red==2 && seed[44:43]==2,"new person episode confirms independently after equipment clears");
        $display("PASS security application: nine colour/type categories x three zones, nine slots, grade priority, exact seeds, source IDs, debounce, ACK/rearm by category, hysteresis, counts and reset");
        $finish;
    end
    initial begin #2000000;$fatal(1,"security application timeout");end
endmodule
