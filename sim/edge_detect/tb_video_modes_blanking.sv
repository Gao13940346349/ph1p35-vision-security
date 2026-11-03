`timescale 1ns/1ps
// Production AWB emits contiguous 4-pixel groups per row, separated by blanks.
// Check every output pixel against an independent 3x3 software-style reference.
// Unlike the old tests, patterns vary in BOTH axes, within groups, and by frame.
module tb_video_modes_blanking #(parameter WIDTH=1280, HEIGHT=12, FULL_SWEEP=1,
    FIRST_MODE=0, LAST_MODE=7);
    localparam GROUPS=WIDTH/4, TOTAL=GROUPS*HEIGHT;
    reg clk=0, rst_n=0, valid=0, last=0, user=0;
    reg [2:0] mode=0;
    reg [95:0] din=0;
    wire ov,ol,ou;
    wire [95:0] dout;
    integer frame=0, pattern_id=0, input_count=0, output_count=0, pixel_checks=0, addr_count=0;
    reg checking=0;
    always #5 clk=~clk;
    video_proc_96 #(.IMG_WIDTH(WIDTH),.IMG_HEIGHT(HEIGHT)) dut(
        .I_clk(clk),.I_rst_n(rst_n),.I_mode(mode),.I_tvalid(valid),
        .I_tlast(last),.I_tuser(user),.I_tdata(din),.I_tready(1'b1),
        .O_tvalid(ov),.O_tlast(ol),.O_tuser(ou),.O_tdata(dout),.O_tready());

    // Check all control cycles, including early frame-start pulses during blanking.
    reg [2:0] control_reference[0:9];
    always @(posedge clk) begin
        if(!rst_n) begin
            for(integer c=0;c<10;c=c+1) control_reference[c]=0;
        end else begin
            for(integer c=9;c>0;c=c-1) control_reference[c]=control_reference[c-1];
            control_reference[0]={user,last,valid};
            #1;
            if({ou,ol,ov}!==control_reference[9]) $fatal(1,"Control latency/alignment changed");
        end
    end

    function automatic [23:0] rgb(input integer f,input integer y,input integer x);
        reg [7:0] r,g,b;
        integer v;
        begin
            case(pattern_id)
                // Smooth two-dimensional gradient, including the former seam.
                0: begin r=16+x*160/WIDTH+y*2+f; g=12+x*140/WIDTH+y*3+f; b=8+x*120/WIDTH+y*4+f; end
                // Non-saturating checker/diagonal patterns exercise intra-group pixels.
                1: begin r=(x%17)*5+(y%5)*9+f; g=(x%13)*7+(y%7)*4+f; b=(x%11)*9+(y%3)*11+f; end
                // Both horizontal and vertical high-contrast edges.
                2: begin r=((x>=WIDTH*3/5) ^ (y>=HEIGHT/2))?255:0; g=r; b=r; end
                // Grain crossing the former per-channel quantizer threshold.
                // Shared RGB offsets retain colour differences in the new fill.
                4: begin v=((x*37+y*57)%7)-3;
                         r=180+v;g=130+v;b=80+v;
                         if(x>=WIDTH/2)begin r=r-100;g=g-100;b=b+100;end end
                // Sensor-like bounded grain around a real, strong colour-card edge.
                default: begin v=80+((x*37+y*57)%21)-10+(x>=WIDTH/2?120:0);
                               r=v; g=v; b=v; end
            endcase
            rgb={r,g,b};
        end
    endfunction
    function automatic integer gray(input integer f,input integer y,input integer x);
        reg [23:0] c;
        begin c=rgb(f,y,x); gray=(77*c[23:16]+150*c[15:8]+29*c[7:0])>>8; end
    endfunction
    integer reference_he[256];
    reg he_reference_pending=0;
    // Observe the map at the HE branch's own first valid input, after its SOF
    // publication. The branch may reuse an existing RGB/control pipeline tap;
    // sampling at the top-level input can then observe the previous frame map.
    // This is a routing check only; the standalone test verifies CDF contents.
    always @(posedge clk) if(rst_n && he_reference_pending && dut.u_he.valid_in) begin
        #2;
        for(integer n=0;n<256;n++) reference_he[n]=dut.u_he.map_valid ?
            dut.u_he.map0[{dut.u_he.active_bank,8'(n)}] : n;
        he_reference_pending=0;
    end
    function automatic [23:0] cartoon_ref(input integer f,y,x,window_ok);
        integer cy,ly,ay,dl,da,sy,target,delta,r,g,b;
        reg [23:0] c;
        begin
            c=rgb(f,y,x);cy=gray(f,y,x);sy=cy;
            dl=0;da=0;
            if(window_ok)begin
                ly=gray(f,y,x-1);ay=gray(f,y-1,x);
                dl=cy>ly ? cy-ly : ly-cy;da=cy>ay ? cy-ay : ay-cy;
                sy=(2*cy+(dl<=12?ly:cy)+(da<=12?ay:cy)+2)/4;
            end
            target=((sy+16)/32)*32;if(target>255)target=255;
            delta=target-cy;
            r=int'(c[23:16])+delta;g=int'(c[15:8])+delta;b=int'(c[7:0])+delta;
            if(r<0)r=0;if(r>255)r=255;
            if(g<0)g=0;if(g>255)g=255;
            if(b<0)b=0;if(b>255)b=255;
            cartoon_ref=window_ok && dl+da>60 ? 24'd0 : {8'(r),8'(g),8'(b)};
        end
    endfunction
    function automatic [23:0] he_ref(input reg [23:0] c);
        integer y,d,r,g,b;
        begin
            y=(int'(c[23:16])+2*int'(c[15:8])+int'(c[7:0]))/4;
            d=reference_he[y]-y;
            r=int'(c[23:16])+d;g=int'(c[15:8])+d;b=int'(c[7:0])+d;
            if(r<0)r=0;if(r>255)r=255;
            if(g<0)g=0;if(g>255)g=255;
            if(b<0)b=0;if(b>255)b=255;
            he_ref={8'(r),8'(g),8'(b)};
        end
    endfunction
    function automatic [23:0] expected(input integer f,input integer y,input integer x);
        integer a,b,c,d,e,g,h,j,k,gx,gy,mag,luma;
        reg [23:0] color;
        begin
            color=rgb(f,y,x);
            // Streaming Sobel uses current row as bottom and previous row as centre.
            if(mode==3 || mode==4) begin
                luma=gray(f,y-1,x);
                a=gray(f,y-2,x-1); b=gray(f,y-2,x); c=gray(f,y-2,x+1);
                d=gray(f,y-1,x-1); e=gray(f,y-1,x); g=gray(f,y-1,x+1);
                h=gray(f,y,x-1);   j=gray(f,y,x);   k=gray(f,y,x+1);
                gx=(c+2*g+k)-(a+2*d+h); gy=(a+2*b+c)-(h+2*j+k);
                mag=(gx<0?-gx:gx)+(gy<0?-gy:gy); if(mag>255) mag=255;
            end else begin luma=gray(f,y,x); mag=0; end
            case(mode)
                1: expected={3{luma[7:0]}};
                2: expected=luma>128?24'hffffff:0;
                3: expected={3{mag[7:0]}};
                4: expected=mag>dut.edge_th_active?24'hffffff:0;
                5: expected=cartoon_ref(f,y,x,1);
                6: expected=~color;
                7: expected=he_ref(color);
                default: expected=color;
            endcase
        end
    endfunction
    integer ox,oy,noise_false_edges=0,noise_true_edges=0;
    integer cartoon_checks=0,cartoon_edges=0,fill_min=255,fill_max=0,old_min=255,old_max=0;
    reg [23:0] original_color;
    reg [23:0] want;
    always @(posedge clk) if(checking && rst_n) begin
        if(dut.u_lb.vld) begin
            if(dut.u_lb.cur_addr!==(addr_count%GROUPS))
                $fatal(1,"Line address: mode=%0d frame=%0d group=%0d actual=%0d",mode,frame,addr_count,dut.u_lb.cur_addr);
            addr_count=addr_count+1;
        end
        if(ov) begin
            ox=output_count%GROUPS; oy=output_count/GROUPS;
            // Existing interface allows frame-start to precede valid (AWB convention).
            if(ol !== (ox==GROUPS-1)) $fatal(1,"Misaligned line-end mode=%0d group=%0d",mode,output_count);
            if(^dout===1'bx) $fatal(1,"Unknown output mode=%0d y=%0d x=%0d",mode,oy,ox*4);
            for(integer p=0;p<4;p=p+1) begin
                // The Sobel window is incomplete at frame/row boundaries.
                // Point modes never inherit that mask; cartoon keeps its fill.
                if((oy<2 || ox<3 || ox>GROUPS-2) && (mode==3 || mode==4))
                    want=0;
                else if((oy<2 || ox<3 || ox>GROUPS-2) && mode==5) begin
                    want=cartoon_ref(frame,oy,ox*4+p,0);
                end else want=expected(frame,oy,ox*4+p);
                if(dout[95-24*p-:24]!==want)
                    $fatal(1,"Pixel mismatch mode=%0d frame=%0d pattern=%0d y=%0d x=%0d actual=%h expected=%h",mode,frame,pattern_id,oy,ox*4+p,dout[95-24*p-:24],want);
                if(mode==4 && pattern_id==3 && oy>=2 && ox>=3 && ox<=GROUPS-2 &&
                   dout[95-24*p-:24]==24'hffffff) begin
                    if(ox*4+p>=WIDTH/2-8 && ox*4+p<=WIDTH/2+8)
                        noise_true_edges=noise_true_edges+1;
                    else
                        noise_false_edges=noise_false_edges+1;
                end
                if(mode==5 && pattern_id==4 && oy>=2 && ox>=3 && ox<=GROUPS-2)begin
                    if(ox*4+p==WIDTH/2)begin
                        if(dout[95-24*p-:24]!==0)$fatal(1,"Cartoon outline shifted from current pixel");
                        cartoon_edges++;
                    end else if(ox*4+p<WIDTH/2-4)begin
                        if(dout[95-24*p-:8]-dout[87-24*p-:8]!=50 ||
                           dout[87-24*p-:8]-dout[79-24*p-:8]!=50)
                            $fatal(1,"Cartoon changed colour differences in flat region");
                        if(dout[87-24*p-:8]<fill_min)fill_min=dout[87-24*p-:8];
                        if(dout[87-24*p-:8]>fill_max)fill_max=dout[87-24*p-:8];
                        original_color=rgb(frame,oy,ox*4+p);
                        if({original_color[15:13],original_color[15:13],original_color[15:14]}<old_min)old_min={original_color[15:13],original_color[15:13],original_color[15:14]};
                        if({original_color[15:13],original_color[15:13],original_color[15:14]}>old_max)old_max={original_color[15:13],original_color[15:13],original_color[15:14]};
                        cartoon_checks++;
                    end
                end
                pixel_checks=pixel_checks+1;
            end
            output_count=output_count+1;
        end
    end
    task automatic send_frame(input integer gap_kind,input integer early_user);
        integer y,x;
        begin
            @(negedge clk); input_count=0; output_count=0; addr_count=0; checking=1;
            he_reference_pending=1;
            if(early_user) begin
                user=1; @(negedge clk); user=0; repeat(30) @(negedge clk);
            end
            for(y=0;y<HEIGHT;y=y+1) begin
                for(x=0;x<GROUPS;x=x+1) begin
                    @(negedge clk); valid=1; user=!early_user && y==0 && x==0; last=x==GROUPS-1;
                    for(integer p=0;p<4;p=p+1) din[95-24*p-:24]=rgb(frame,y,x*4+p);
                    input_count=input_count+1;
                end
                if(gap_kind>0) begin
                    @(negedge clk); valid=0; user=0; last=0; din=96'h123456abcdef987654fedcba;
                    // One clock, or real AWB-like 680/1000-clock row blanking.
                    if(gap_kind==2) repeat((y%2==0)?679:999) @(negedge clk);
                end
            end
            @(negedge clk); valid=0; user=0; last=0; din=0;
            repeat(30) @(negedge clk); checking=0;
            if(output_count!=TOTAL || input_count!=TOTAL || addr_count!=TOTAL)
                $fatal(1,"Frame length mode=%0d in=%0d out=%0d cached=%0d",mode,input_count,output_count,addr_count);
        end
    endtask
    task automatic send_partial_lines(input integer rows);
        begin
            for(integer y=0;y<rows;y=y+1) begin
                for(integer x=0;x<GROUPS;x=x+1) begin
                    @(negedge clk); valid=1; user=0; last=x==GROUPS-1;
                    for(integer p=0;p<4;p=p+1)
                        din[95-24*p-:24]=rgb(frame,y,x*4+p);
                end
                @(negedge clk); valid=0; last=0; din=0;
                repeat(4) @(negedge clk);
            end
            repeat(30) @(negedge clk);
        end
    endtask
    initial begin
        repeat(5) @(negedge clk); rst_n=1;
        for(integer m=FIRST_MODE;m<=LAST_MODE;m=m+1) begin
            mode=m;
            for(integer pat=(FULL_SWEEP?0:1);pat<(FULL_SWEEP?3:2);pat=pat+1) begin
                pattern_id=pat;
                for(integer gap=(FULL_SWEEP?0:2);gap<3;gap=gap+1) begin
                    // Consecutive frames without resetting DUT, both frame-start forms.
                    frame=0; send_frame(gap,0);
                    frame=1; send_frame(gap,1);
                end
            end
            $display("MODE_PASS mode=%0d exact pixel checks=%0d",mode,pixel_checks);
        end
        mode=4; pattern_id=3; frame=0;
        send_frame(2,1);
        if(noise_false_edges!=0 || noise_true_edges==0)
            $fatal(1,"Noisy edge mask failed: false=%0d true=%0d",noise_false_edges,noise_true_edges);
        $display("PASS noisy flat field suppressed, strong edge retained: true=%0d",noise_true_edges);
        mode=5;pattern_id=4;frame=0;send_frame(2,0);
        if(cartoon_checks==0 || cartoon_edges!=HEIGHT-2 || fill_max-fill_min>=old_max-old_min)
            $fatal(1,"Cartoon quality check failed: checks=%0d edges=%0d fill=%0d old=%0d",cartoon_checks,cartoon_edges,fill_max-fill_min,old_max-old_min);
        $display("PASS cartoon: colour preserved, outline at current pixel, flat green variation new=%0d old=%0d checks=%0d",fill_max-fill_min,old_max-old_min,cartoon_checks);
        // Camera/ISP reset may release in the middle of a frame. The next
        // frame-start pulse can arrive during blanking, before valid rises.
        // The first complete frame must then start its window mask at row 0.
        mode=3; pattern_id=1; frame=0;
        send_partial_lines(5);
        send_frame(2,1);
        $display("PASS early frame-start resync after partial frame");
        $display("PASS video_modes_blanking width=%0d height=%0d pixels=%0d full_sweep=%0d modes=%0d..%0d, two frame-start forms",WIDTH,HEIGHT,pixel_checks,FULL_SWEEP,FIRST_MODE,LAST_MODE);
        $finish;
    end
    initial begin #1000000000; $fatal(1,"Mode blanking test timeout"); end
endmodule
