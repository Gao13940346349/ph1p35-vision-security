`timescale 1ns/1ps
module tb_color_shapes #(parameter W=1280,H=720,FILTER=1);
    localparam MW=W/4,MH=H/4,N=MW*MH;
    reg clk=0;always #4.444 clk=~clk;
    reg rst_n=0,enable=1,sof=0,valid=0;
    reg [95:0] rgb=0;
    wire done,busy,overflow;wire [15:0] id;wire [422:0] boxes;
    color_shape_detector #(.IMG_WIDTH(W),.IMG_HEIGHT(H),.SPATIAL_FILTER(FILTER)) dut(
        .clk(clk),.rst_n(rst_n),.enable(enable),.sof(sof),.valid(valid),.rgb(rgb),
        .result_strobe(done),.result_id(id),.boxes(boxes),.busy(busy),.overflow(overflow));
    integer owner[0:N-1],count=0,frame=0;
    integer colour[0:15],kind[0:15],xc[0:15],yc[0:15],radius[0:15];
    integer area[0:15],xmin[0:15],xmax[0:15],ymin[0:15],ymax[0:15],noise_count;
    bit seen[0:15];
    real angle[0:15];
    real scale_x=1.0, shear_x=0.0;
    function automatic bit in_shape(input integer t,x,y);
        real u,v,c,s;
        begin
            c=$cos(angle[t]*3.14159265358979/180.0);s=$sin(angle[t]*3.14159265358979/180.0);
            u=((x-xc[t])-shear_x*(y-yc[t]))/scale_x*c+(y-yc[t])*s;
            v=-((x-xc[t])-shear_x*(y-yc[t]))/scale_x*s+(y-yc[t])*c;
            case(kind[t])
                0:in_shape=v>=-radius[t] && v<=radius[t]*0.5 &&
                    u<=0.577350269*(v+radius[t]) && u>=-0.577350269*(v+radius[t]);
                1:in_shape=u>=-radius[t] && u<=radius[t] && v>=-radius[t] && v<=radius[t];
                2:in_shape=u*u+v*v<=radius[t]*radius[t];
                default:in_shape=0;
            endcase
        end
    endfunction
    integer palette_mode=0;
    function automatic [23:0] pixel(input integer c);
        case(palette_mode)
            1: case(c) 1:pixel=24'h807064;2:pixel=24'h648070;3:pixel=24'h706480;default:pixel=24'h666462;endcase
            2: case(c) 1:pixel=24'h24180c;2:pixel=24'h0c2418;3:pixel=24'h180c24;default:pixel=24'h181818;endcase
            default: case(c) 1:pixel=24'hcc2028;2:pixel=24'h28cc20;3:pixel=24'h2028cc;default:pixel=24'h606060;endcase
        endcase
    endfunction
    task automatic clear_scene;
        begin count=0;for(int p=0;p<N;p++)owner[p]=-1;end
    endtask
    task automatic add(input integer c,s,x,y,r,input real a);
        begin colour[count]=c;kind[count]=s;xc[count]=x;yc[count]=y;radius[count]=r;angle[count]=a;count++;end
    endtask
    task automatic paint;
        begin
            for(int t=0;t<count;t++) begin
                area[t]=0;xmin[t]=MW;ymin[t]=MH;xmax[t]=0;ymax[t]=0;seen[t]=0;
                for(int y=0;y<MH;y++) for(int x=0;x<MW;x++) if(in_shape(t,x,y)) begin
                    if(owner[y*MW+x]!=-1)$fatal(1,"test geometry overlap");
                    owner[y*MW+x]=t;area[t]++;
                    if(x<xmin[t])xmin[t]=x;if(x>xmax[t])xmax[t]=x;
                    if(y<ymin[t])ymin[t]=y;if(y>ymax[t])ymax[t]=y;
                end
            end
        end
    endtask
    task automatic run_frame(input bit expect_overflow=0,input bit tie=0,input bit coincident=0);
        integer o,c,cycles,n,match,min_selected;
        reg [46:0] b;
        begin
            for(int t=0;t<count;t++)seen[t]=0;
            if(FILTER) begin
                // Independent spatial oracle on object ownership, not DUT internals.
                for(int t=0;t<count;t++) begin
                    xmin[t]=MW;ymin[t]=MH;xmax[t]=0;ymax[t]=0;area[t]=0;
                    for(int y=1;y<MH;y++)for(int x=1;x<MW;x++) begin
                        c=(owner[y*MW+x]==t)+(owner[y*MW+x-1]==t)+
                          (owner[(y-1)*MW+x]==t)+(owner[(y-1)*MW+x-1]==t);
                        if(c>=3) begin
                            area[t]++;
                            if(x<xmin[t])xmin[t]=x;if(x>xmax[t])xmax[t]=x;
                            if(y<ymin[t])ymin[t]=y;if(y>ymax[t])ymax[t]=y;
                        end
                    end
                end
            end
            frame++;@(negedge clk);sof=1;valid=coincident;rgb={4{pixel(0)}};
            @(negedge clk);sof=0;valid=0;if(!coincident)repeat(3)@(negedge clk);
            for(int y=0;y<H;y++) begin
                for(int x=(coincident && y==0 ? 1 : 0);x<MW;x++) begin
                    o=owner[(y/4)*MW+x];c=o<0 ? 0 : colour[o];
                    valid=1;rgb=tie ? {24'hff0000,24'hff0000,24'h0000ff,24'h0000ff} : {4{pixel(c)}};
                    @(negedge clk);
                    if(frame%2==0 && x%31==17)begin valid=0;repeat(2)@(negedge clk);end
                end
                valid=0;repeat(7)@(negedge clk);
            end
            valid=0;cycles=0;
            while(!done && cycles<N*30)begin @(negedge clk);cycles++;end
            if(!done || id!=frame || overflow!=expect_overflow)$fatal(1,"frame %0d completion/id/overflow got %0d/%b/%b",frame,id,done,overflow);
            n=0;min_selected=N;
            for(int k=0;k<9;k++) begin
                b=boxes[k*47 +:47];
                if(b[42])begin
                    n++;match=-1;
                    for(int t=0;t<count;t++)
                        if(b[41:31]==xmin[t]*4 && b[30:20]==xmax[t]*4+3 &&
                           b[19:10]==ymin[t]*4 && b[9:0]==ymax[t]*4+3)match=t;
                    if(match<0)$fatal(1,"frame %0d unknown box %h",frame,b);
                    if(seen[match] || b[46:45]!=colour[match] || b[44:43]!=kind[match])
                        $fatal(1,"frame %0d object %0d classification got colour=%0d shape=%0d expected %0d/%0d",frame,match,b[46:45],b[44:43],colour[match],kind[match]);
                    seen[match]=1;if(area[match]<min_selected)min_selected=area[match];
                end
            end
            if(n!=(tie ? 0 : count>9 ? 9 : count))begin
                for(int t=0;t<count;t++)if(!seen[t])$display("missing object %0d c/s=%0d/%0d angle=%f area=%0d",t,colour[t],kind[t],angle[t],area[t]);
                $fatal(1,"frame %0d count got %0d expected %0d",frame,n,count>9 ? 9 : count);
            end
            if(!expect_overflow && !tie) for(int t=0;t<count;t++)
                if(!seen[t] && area[t]>min_selected)$fatal(1,"larger object omitted from top nine");
            $display("scene %0d PASS objects=%0d process_cycles=%0d",frame,n,cycles);
            repeat(4)@(negedge clk);
        end
    endtask
    initial begin
        repeat(6)@(negedge clk);rst_n=1;
        // All nine colour/shape combinations simultaneously, independently positioned.
        clear_scene();
        for(int c=1;c<=3;c++)for(int s=0;s<3;s++)add(c,s,(s*2+1)*MW/6,(c*2-1)*MH/6,14,0);
        paint();run_frame();
        // Repeat exactly stationary geometry under washed-out and dark colours.
        for(int p=1;p<=2;p++)begin palette_mode=p;repeat(3)run_frame();end
        palette_mode=0;
        // Repeated classes, square rotation (including diamond), triangle rotation.
        for(int turn=0;turn<4;turn++)begin
            clear_scene();
            for(int t=0;t<9;t++)add(1+t%3,t/3,(t%3*2+1)*MW/6,(t/3*2+1)*MH/6,14,turn*22.5+15);
            paint();run_frame(0,0,turn%2);
        end
        clear_scene();
        for(int t=0;t<9;t++)add(1,2,(t%3*2+1)*MW/6,(t/3*2+1)*MH/6,14,0);
        paint();run_frame();
        // More than nine objects of one class; top nine by area, not by class.
        clear_scene();
        for(int t=0;t<12;t++)add(3,1,(t%4*2+1)*MW/8,(t/4*2+1)*MH/6,8+t,0);
        paint();run_frame();
        // Small colour regions fail the new default size/area thresholds.
        clear_scene();add(1,1,MW/2,MH/2,14,0);
        add(2,1,30,30,5,0);add(3,2,65,30,7,0);
        paint();count=1;run_frame();
        // Different colours in direct contact must not merge into one component.
        clear_scene();add(1,1,MW/2-14,MH/2,13,0);add(2,1,MW/2+13,MH/2,13,0);
        paint();run_frame();
        clear_scene();paint();run_frame();run_frame(0,1);
        // A static target later in raster order must survive hundreds of sharp,
        // isolated colour fragments. This exceeded the former 127-label table.
        clear_scene();add(2,2,MW*3/4,MH*3/4,14,0);paint();colour[15]=1;
        noise_count=0;
        for(int y=2;y<MH-2;y+=3)for(int x=2;x<MW-2;x+=3)
            if(owner[y*MW+x]<0 && noise_count<800)begin owner[y*MW+x]=15;noise_count++;end
        repeat(5)run_frame();
        // More than 1023 toolbar-like fragments above three targets. Empty
        // rows permit recycling; targets must survive modest skew/foreshortening.
        for(int turn=0;turn<4;turn++) begin
            clear_scene();scale_x=0.85;shear_x=0.12;
            for(int t=0;t<3;t++)add(1+t,t,(t*2+1)*MW/6,MH*3/4,20,turn*15.0);
            paint();colour[15]=2;
            for(int y=2;y<50;y+=3)for(int x=2;x<MW-3;x+=4)
                begin owner[y*MW+x]=15;owner[y*MW+x+1]=15;end
            run_frame();
        end
        scale_x=1.0;shear_x=0.0;
        // 1024+ two-cell fragments do exhaust the bounded table, but a valid
        // component found first remains visible instead of clearing the frame.
        clear_scene();add(3,1,MW/2,20,14,0);paint();colour[15]=2;
        for(int y=45;y<MH-2;y+=3)for(int x=2;x<MW-3;x+=4)
            if(owner[y*MW+x]<0 && owner[y*MW+x+1]<0)begin owner[y*MW+x]=15;owner[y*MW+x+1]=15;end
        // Border stripe prevents empty-row recycling in this capacity test.
        for(int y=45;y<MH;y++)owner[y*MW]=15;
        run_frame(FILTER ? 0 : 1,0);
        // No blank rows: >2000 fragments plus a continuous coloured edge.
        // Previously exhausted the label table BEFORE reaching dominant objects.
        if(FILTER) begin
            clear_scene();
            add(1,2,MW/3,MH*3/4,20,0);add(3,1,MW*3/4,MH*3/4,20,15);
            paint();colour[15]=2;
            for(int y=2;y<MH/2;y+=2)for(int x=2;x<MW-2;x+=3) owner[y*MW+x]=15;
            for(int y=0;y<MH;y++)owner[y*MW]=15;
            repeat(6)run_frame();
        end
        // Disabled SOF still advances source identity, but must not launch work.
        enable=0;frame++;@(negedge clk);sof=1;valid=1;
        @(negedge clk);sof=0;
        repeat(500)begin @(negedge clk);if(done || busy)$fatal(1,"disabled detector started work");end
        valid=0;enable=1;clear_scene();add(3,2,MW/2,MH/2,14,0);paint();run_frame(0,0,1);
        // Reset clears the result and identity; stale mask RAM must not leak.
        rst_n=0;repeat(5)@(negedge clk);if(boxes!==0)$fatal(1,"reset result not empty");rst_n=1;frame=0;
        clear_scene();paint();run_frame();
        $display("PASS color_shapes static clutter, fragment overflow preservation, nine classes, rotations, repeats, top nine, touching colours, gaps, SOF, disable, reset");$finish;
    end
    initial begin #200000000;$fatal(1,"watchdog");end
endmodule
