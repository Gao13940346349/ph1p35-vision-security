`timescale 1ns/1ps
module tb_shape_overlay #(parameter integer SHOW_COORDINATES=1, BORDER_WIDTH=3);
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,de=0;reg [11:0] x=0,y=0;reg [422:0] boxes=0;
    wire hit;wire [23:0] colour;
    shape_overlay #(.SHOW_COORDINATES(SHOW_COORDINATES),.BORDER_WIDTH(BORDER_WIDTH)) dut(.clk(clk),.rst_n(rst_n),.de(de),.x(x),.y(y),.boxes(boxes),.hit(hit),.colour(colour));
    integer cx[0:8],cy[0:8],ex[0:8],ey[0:8],c[0:8],s[0:8];
    bit v[0:8];
    integer pixels=0,red=0,green=0,blue=0,letters=0;
    reg [24:0] previous=0;
    // Golden character table uses ASCII from the literal words, independently
    // of the RTL's per-shape/per-index decoder.
    function automatic [63:0] font(input byte ch);
        case(ch)
            "a":font=64'h000038043c443c00;"c":font=64'h00003c4040403c00;
            "e":font=64'h000038447c403c00;"g":font=64'h00003c44443c0478;
            "i":font=64'h1000301010103800;"l":font=64'h3010101010103800;
            "n":font=64'h0000586444444400;"q":font=64'h00003c44443c0406;
            "r":font=64'h0000586440404000;"s":font=64'h00003c4038047800;
            "t":font=64'h10103c1010141800;"u":font=64'h00004444444c3400;
            "0":font=64'h0038444c54443800;"1":font=64'h0010301010103800;
            "2":font=64'h0038440810207c00;"3":font=64'h0078083008443800;
            "4":font=64'h00081828487c0800;"5":font=64'h007c407804443800;
            "6":font=64'h0038407844443800;"7":font=64'h007c040810202000;
            "8":font=64'h0038443844443800;"9":font=64'h003844443c043800;
            "(":font=64'h0008102020100800;")":font=64'h0020100808102000;
            ",":font=64'h0000000000181830;
            default:font=0;
        endcase
    endfunction
    function automatic [24:0] golden(input integer px,py,input bit active);
        integer chosen,lx,ly,w,col,row,mx,my;
        reg [63:0] bits;
        string word_text;
        begin
            chosen=-1;
            for(int k=0;k<9;k++) if(active && v[k] && s[k]<3 && c[k]!=0 && chosen<0)begin
                case(s[k])0:word_text="triangle";1:word_text="square";2:word_text="circle";endcase
                w=160;lx=cx[k]+w>1280 ? 1280-w : cx[k];ly=cy[k]>=32 ? cy[k]-32 : cy[k]+3;
                mx=(cx[k]+ex[k])/2;my=(cy[k]+ey[k])/2;
                if(SHOW_COORDINATES && py>=ly+16)word_text=$sformatf("(%0d%0d%0d%0d,%0d%0d%0d)",
                    mx/1000,mx/100%10,mx/10%10,mx%10,my/100,my/10%10,my%10);
                if(px>=lx && px<lx+w && py>=ly && py<ly+(SHOW_COORDINATES ? 32 : 16) && px<1280 && py<720)begin
                    col=(px-lx)%16/2;row=(py-ly)%16/2;
                    bits=(px-lx)/16<word_text.len() ? font(word_text[(px-lx)/16]) : 64'd0;
                    if(bits[63-row*8-col])chosen=k;
                end
            end
            for(int k=0;k<9;k++) if(active && v[k] && s[k]<3 && c[k]!=0 && chosen<0 &&
                px>=cx[k] && px<=ex[k] && py>=cy[k] && py<=ey[k] &&
                (px<cx[k]+BORDER_WIDTH || px>ex[k]-BORDER_WIDTH ||
                 py<cy[k]+BORDER_WIDTH || py>ey[k]-BORDER_WIDTH))chosen=k;
            golden=0;
            if(chosen>=0)case(c[chosen])1:golden={1'b1,24'hff0000};2:golden={1'b1,24'h00ff00};3:golden={1'b1,24'h0000ff};endcase
        end
    endfunction
    task automatic tick(input integer px,py,input bit active);
        reg [24:0] expected;
        begin
            @(negedge clk);x=px;y=py;de=active;expected=golden(px,py,active);
            @(posedge clk);#1;
            if({hit,colour}!==previous)$fatal(1,"overlay latency/pixel mismatch near %0d,%0d got %h expected %h",px,py,{hit,colour},previous);
            previous=expected;pixels++;
            if(hit)case(colour)24'hff0000:red++;24'h00ff00:green++;24'h0000ff:blue++;endcase
        end
    endtask
    task automatic pack_boxes;
        begin
            de=0;
            for(int k=0;k<9;k++)boxes[k*47 +:47]={2'(c[k]),2'(s[k]),v[k],11'(cx[k]),11'(ex[k]),10'(cy[k]),10'(ey[k])};
            // Descriptor updates happen in vertical blanking; allow one full
            // shared conversion sweep, including an in-flight old snapshot.
            repeat(260)@(negedge clk);
            previous=0;
        end
    endtask
    initial begin
        repeat(5)@(negedge clk);rst_n=1;
        for(int k=0;k<9;k++)begin
            cx[k]=60+(k%3)*400;cy[k]=40+(k/3)*210;ex[k]=cx[k]+70;ey[k]=cy[k]+70;
            c[k]=1+k/3;s[k]=k%3;v[k]=1;
        end
        // Top-edge fallback, right-edge text shift, and intersecting boxes/text.
        cy[0]=4;ey[0]=74;cx[3]=1200;ex[3]=1279;
        cx[4]=cx[5]-16;cy[4]=cy[5]+10;ex[4]=ex[5];ey[4]=ey[5]+10;
        pack_boxes();
        for(int py=0;py<720;py++)begin
            for(int px=0;px<1280;px++)tick(px,py,1);
            tick(1280,py,0);tick(1281,py,0);
        end
        tick(0,0,0);tick(0,0,0);
        if(red<100 || green<100 || blue<100)$fatal(1,"missing colour coverage");
        // All invalid descriptors must be transparent even with nonzero coords.
        for(int k=0;k<9;k++)v[k]=0;pack_boxes();
        for(int py=0;py<100;py++)for(int px=0;px<200;px++)tick(px,py,1);
        tick(0,0,0);tick(0,0,0);
        $display("PASS shape_overlay %0d pixels: nine coloured boxes, literal lowercase labels, two-cycle alignment, clipping, overlaps, invalid descriptors; SHOW_COORDINATES=%0d BORDER_WIDTH=%0d",pixels,SHOW_COORDINATES,BORDER_WIDTH);$finish;
    end
    initial begin #20000000;$fatal(1,"watchdog");end
endmodule
