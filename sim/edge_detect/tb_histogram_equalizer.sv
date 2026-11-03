`timescale 1ns/1ps
module tb_histogram_equalizer;
    localparam W=64,H=32,G=W/4,STRIDE=4;
    reg clk=0,rst_n=0,en=1,v=0,l=0,u=0;
    reg [95:0] din=0;
    wire [95:0] dout;
    wire ov,ol,ou;
    always #5 clk=~clk;
    luma_histogram_equalizer_96 #(.IMG_WIDTH(W),.IMG_HEIGHT(H),.SAMPLE_STRIDE(STRIDE)) dut(
        .clk(clk),.rst_n(rst_n),.enable(en),.data_in(din),.valid_in(v),.last_in(l),.user_in(u),
        .data_out(dout),.valid_out(ov),.last_out(ol),.user_out(ou));
    integer gold[256],hist[256],new_gold[256];
    integer sample_lane=0,checks=0,frames=0;
    reg [95:0] reference_data[5];
    reg [2:0] reference_ctl[5];
    function automatic integer y_of(input reg [23:0] c);
        y_of=(int'(c[23:16])+2*int'(c[15:8])+int'(c[7:0]))/4;
    endfunction
    function automatic [23:0] colour(input integer pattern,y,g,p);
        integer b;
        begin
            if(pattern==0) colour=0;
            else if(pattern==1) colour=24'h808080;
            else begin
                b=80+(g*7+y*13+p*3)%80;
                colour={8'(b+20),8'(b),8'(b-20)};
            end
        end
    endfunction
    function automatic [23:0] apply_table(input reg [23:0] c);
        integer delta,r,g,b;
        begin
            delta=en ? gold[y_of(c)]-y_of(c) : 0;
            r=int'(c[23:16])+delta;g=int'(c[15:8])+delta;b=int'(c[7:0])+delta;
            if(r<0)r=0;if(r>255)r=255;
            if(g<0)g=0;if(g>255)g=255;
            if(b<0)b=0;if(b>255)b=255;
            apply_table={8'(r),8'(g),8'(b)};
        end
    endfunction
    always @(posedge clk) begin
        if(!rst_n) begin
            for(integer n=0;n<5;n++) begin reference_ctl[n]=0;reference_data[n]=0;end
        end else begin
            for(integer n=4;n>0;n--) begin
                reference_ctl[n]=reference_ctl[n-1];reference_data[n]=reference_data[n-1];
            end
            reference_ctl[0]={u,l,v};
            for(integer p=0;p<4;p++) reference_data[0][95-24*p -:24]=apply_table(din[95-24*p -:24]);
            #1;
            if({ou,ol,ov}!==reference_ctl[4]) $fatal(1,"HE control alignment");
            if(ov) begin
                if(dout!==reference_data[4]) $fatal(1,"HE pixels actual=%h expected=%h",dout,reference_data[4]);
                checks+=4;
            end
        end
    end
    task automatic frame(input integer pattern,early,collect_reference,bad_line=0);
        integer y,g,p,n;
        begin
            if(collect_reference) for(n=0;n<256;n++) hist[n]=0;
            @(negedge clk);
            if(early) begin u=1;@(negedge clk);u=0;repeat(3)@(negedge clk);end
            for(y=0;y<H;y++) begin
                for(g=0;g<G;g++) begin
                    @(negedge clk);v=1;l=(g==G-1 || (bad_line && y==0 && g==2));u=!early && y==0 && g==0;
                    for(p=0;p<4;p++) din[95-24*p -:24]=colour(pattern,y,g,p);
                    if(collect_reference && (y*G+g)%STRIDE==0) begin
                        n=y_of(colour(pattern,y,g,sample_lane));hist[n]++;
                        sample_lane=(sample_lane+1)%4;
                    end
                end
                @(negedge clk);v=0;l=0;u=0;
                repeat(y%3) @(negedge clk);
            end
            frames++;
        end
    endtask
    task automatic calculate_reference;
        integer total,first,sum,d,m,nonzero;
        begin
            total=0;nonzero=0;
            for(integer n=0;n<256;n++) begin total+=hist[n];if(hist[n])nonzero++;end
            first=0;sum=0;
            for(integer n=0;n<256;n++) begin
                sum+=hist[n];
                if(first==0 && hist[n]!=0) first=sum;
                m=(sum<=first || first==0 || total==first) ? 0 : (sum-first)*255/(total-first);
                d=m-n;if(d>64)d=64;if(d< -64)d= -64;
                new_gold[n]=nonzero==1 ? n : n+(d>>>1);
            end
        end
    endtask
    task automatic wait_and_verify_table;
        integer watchdog;
        begin
            watchdog=0;
            while(!dut.map_pending && watchdog<20000) begin @(negedge clk);watchdog++;end
            if(!dut.map_pending)$fatal(1,"CDF build timed out");
            for(integer n=0;n<256;n++) begin
                if(dut.map0[{dut.build_bank,8'(n)}]!==8'(new_gold[n]))
                    $fatal(1,"CDF LUT bin=%0d got=%0d expected=%0d",n,dut.map0[{dut.build_bank,8'(n)}],new_gold[n]);
                if(n>0 && new_gold[n]<new_gold[n-1])$fatal(1,"Nonmonotonic map");
            end
        end
    endtask
    task automatic publish_reference;
        for(integer n=0;n<256;n++)gold[n]=new_gold[n];
    endtask
    initial begin
        for(integer n=0;n<256;n++)gold[n]=n;
        repeat(5)@(negedge clk);rst_n=1;repeat(300)@(negedge clk);
        frame(0,0,1);calculate_reference();wait_and_verify_table();publish_reference();
        frame(1,1,1);calculate_reference();wait_and_verify_table();publish_reference();
        frame(2,0,1);calculate_reference();
        // No build wait: next whole video frame must run with the old LUT.
        frame(2,1,0);wait_and_verify_table();publish_reference();
        if(new_gold[100]==100 || new_gold[140]==140)$fatal(1,"HE did not alter compressed contrast");
        frame(2,0,1);calculate_reference();wait_and_verify_table();publish_reference();
        en=0;frame(2,1,1);calculate_reference();wait_and_verify_table();publish_reference();
        en=1;frame(2,0,1);repeat(12)@(negedge clk);
        // Reset invalidates both old maps and histogram contents, independently
        // of stale RAM bits. First complete video is identity again.
        rst_n=0;v=0;u=0;l=0;
        repeat(5)@(negedge clk);
        for(integer n=0;n<256;n++)gold[n]=n;
        sample_lane=0;rst_n=1;repeat(300)@(negedge clk);
        frame(2,0,1);calculate_reference();wait_and_verify_table();
        publish_reference();
        // An incomplete frame must never publish a partial CDF. A new SOF
        // invalidates capture, then a complete subsequent frame can recover.
        @(negedge clk);v=0;l=0;u=1;@(negedge clk);u=0;
        for(integer g=0;g<7;g++)begin
            @(negedge clk);v=1;din={4{24'h604020}};
        end
        @(negedge clk);v=0;u=1;@(negedge clk);u=0;
        sample_lane=(sample_lane+2)%4;
        repeat(300)@(negedge clk);
        if(dut.map_pending || dut.state!=1)$fatal(1,"Partial frame histogram was not discarded");
        frame(2,1,1);calculate_reference();wait_and_verify_table();publish_reference();
        frame(2,0,0,1);repeat(300)@(negedge clk);
        if(dut.map_pending || dut.state!=1)$fatal(1,"Malformed line histogram was not discarded");
        frame(2,0,1);calculate_reference();wait_and_verify_table();
        repeat(10)@(negedge clk);
        $display("PASS histogram_equalizer: CDF-min all 256 bins, colour pixels=%0d, frames=%0d, flat/black/bypass/short blank/reset/partial SOF/bad line recovery",checks,frames);
        $finish;
    end
    initial begin #10000000;$fatal(1,"HE timeout");end
endmodule
