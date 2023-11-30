`timescale 1ns / 1ps

module vgas(
clock,reset,         //inputs - sel_module(select required function), reset(to switch on and off), val(give a value to adjust brightness and filters)
hsync,vsync, sel,                        // hsync and vsync for the working of monitor
red, green, blue                    // red, green and blue output pixels
);

    input clock;
    input reset;
    input [2:0] sel;
    reg [2:0] temp_sel, select;
    reg [7:0] gray, left, right, up, down, leftup, leftdown, rightup, rightdown;       //different values in matrix
    reg[7:0] red_o, blue_o, green_o;            // variables used during calcultion

   reg clk;
   initial begin
   clk =0;
   end
   always@(posedge clock)
   begin
    clk<=~clk;
   end

	output reg hsync;
   output reg vsync;
   reg [7:0] tred,tgreen,tblue;
	output reg [3:0] red,green;
	output reg [3:0] blue;

 
	reg read = 0;
	reg [15:0] r, g, b;
	reg [14:0] addra = 0;
	reg [14:0] addrb = 0;
	reg [95:0] in1 = 0;
	wire [95:0] out2, out;
	
	wire [95:0] din = 96'b0;
	wire [95:0] dinb = 96'b0;



    blk_mem_gen_2 read_rom (
  .clka(clk),    // input wire clka
  .wea(1'b0),      // input wire [0 : 0] wea
  .addra(addra),  // input wire [14 : 0] addra
  .dina(din),    // input wire [95 : 0] dina
  .douta(out2),  // output wire [95 : 0] douta
  .clkb(clk), 
   .enb(1'b1),     // input wire clkb
  .web(1'b0),     // input wire [0 : 0] web
  .addrb(addrb),  // input wire [14 : 0] addrb
  .dinb(dinb),    // input wire [95 : 0] dinb
  .doutb(out)  // output wire [95 : 0] doutb
);


    reg [14:0] addra_2 =0;
    reg [14:0] addrb_2 =0;
    reg [7:0] in;
    wire [7:0] out_3;
    wire [7:0] dout;
    wire [7:0] din_b;
    
    

    blk_mem_gen_1 your_instance_name (
  .clka(clock),    // input wire clka
  .wea(1'b1),      // input wire [0 : 0] wea
  .addra(addra_2),  // input wire [14 : 0] addra
  .dina(in),    // input wire [7 : 0] dina
  .douta(dout),  // output wire [7 : 0] douta
  .clkb(clk),    // input wire clkb
  .web(1'b0),      // input wire [0 : 0] web
  .addrb(addrb_2),  // input wire [14 : 0] addrb
  .dinb(din_b),    // input wire [7 : 0] dinb
  .doutb(out_3)  // output wire [7 : 0] doutb
);


    wire [15:0] out_m;
    reg res;
    reg [7:0] input_m;
    wire done_1;

    multiplier m(
	.in1(input_m),
	.in2(8'd2),
	.start(1'b1), 
	.reset(res),
	.clk(clk),

	.out(out_m),
	.done(done_1)
);

    reg flag =0;
    
    always @(posedge clk && !flag) begin
        
       
        input_m = out[79:72];
        if (done_1 == 1) begin
        
            in = (out_m>255)?255:out_m[7:0];    
            
            if(addra_2 <18239) begin
            
            addra_2 = addra_2 +1;
            end
            else begin
            
                addra_2 <=0;
                flag = 1;
            end
            
            if(addrb<18239) begin
            
            addrb = addrb +1;
            end
            else begin
            
                addrb =0;
            end
        
        
        
    
        end
    
    end




	wire pixel_clk;
   reg 		pcount = 0;
   wire 	ec = (pcount == 0);
   always @ (posedge clk) pcount <= ~pcount;
   assign 	pixel_clk = ec;
   
   reg 		hblank=0,vblank=0;
   initial begin
   hsync =0;
   vsync=0;
   end
   reg [9:0] 	hc=0;      
   reg [9:0] 	vc=0;	 
	
   wire 	hsyncon,hsyncoff,hreset,hblankon;
   assign 	hblankon = ec & (hc == 639);    
   assign 	hsyncon = ec & (hc == 655);
   assign 	hsyncoff = ec & (hc == 751);
   assign 	hreset = ec & (hc == 799);
   
   wire 	blank =  (vblank | (hblank & ~hreset));    
   
   wire 	vsyncon,vsyncoff,vreset,vblankon;
   assign 	vblankon = hreset & (vc == 479);
  
   assign 	vsyncon = hreset & (vc == 490);
   assign 	vsyncoff = hreset & (vc == 492);
   assign 	vreset = hreset & (vc == 523);

   always @(posedge clk) begin
   hc <= ec ? (hreset ? 0 : hc + 1) : hc;
   hblank <= hreset ? 0 : hblankon ? 1 : hblank;
   hsync <= hsyncon ? 0 : hsyncoff ? 1 : hsync; 
   
   vc <= hreset ? (vreset ? 0 : vc + 1) : vc;
   vblank <= vreset ? 0 : vblankon ? 1 : vblank;
   vsync <= vsyncon ? 0 : vsyncoff ? 1 : vsync;
    end

    always @(posedge clk) begin
    if (flag) begin
    if(hc==0) begin
    
    select <= sel;
    
    end
    end
    
    end


always @(posedge pixel_clk)
	begin		
	
	    if(flag) begin
            


           
                         
            if(blank == 0 && hc >= 'd0 && hc < 'd160 && vc >= 'd000 && vc < 'd114)
            begin
              
                gray =  {out2[95], out2[94], out2[93], out2[92], out2[91], out2[90], out2[89], out2[88]};
                left = {out2[87], out2[86], out2[85], out2[84], out2[83], out2[82], out2[81], out2[80]};
                right = {out2[79], out2[78], out2[77], out2[76], out2[75], out2[74], out2[73], out2[72]};                
                            
                up =  {out2[71], out2[70], out2[69], out2[68], out2[67], out2[66], out2[65], out2[64]};
                down = {out2[63], out2[62], out2[61], out2[60], out2[59], out2[58], out2[57], out2[56]};
                leftup = {out2[55], out2[54], out2[53], out2[52], out2[51], out2[50], out2[49], out2[48]};
                leftdown =  {out2[47], out2[46], out2[45], out2[44], out2[43], out2[42], out2[41], out2[40]};
                rightup = {out2[39], out2[38], out2[37], out2[36], out2[35], out2[34], out2[33], out2[32]};
                rightdown = {out2[31], out2[30], out2[29], out2[28], out2[27], out2[26], out2[25], out2[24]};
                tblue =  {out2[23], out2[22], out2[21], out2[20], out2[19], out2[18], out2[17], out2[16]};
                tgreen = {out2[15], out2[14], out2[13], out2[12], out2[11], out2[10], out2[9], out2[8]};
                tred = {out2[7], out2[6], out2[5], out2[4], out2[3], out2[2], out2[1], out2[0]};
                
                
                if(hc == 'd0 &&  vc == 'd0) begin
                    addra = 0;
                    
                end
                
                

//                 RGB image to gray scale image
              
                    if(reset) begin
                        red = 0;
                        green = 0;
                        blue = 0;
                        
                    end 
                    
                    else if (select == 'b000) begin
                    

                            r = ((rightup)- leftup + (out_3) - (2*left) + rightdown - leftdown);
                       g = ((rightup) + (2*up) + leftup - rightdown - (2*down) - leftdown);
                       
                       if(r > 1024 & g > 1024)begin
                           b = -(r + g)/2;
                       end else if(r > 1024 & g < 1024)begin
                           b = (-r  + g)/2;
                       end else if(r < 1024 & g < 1024)begin
                           b = (r + g)/2;
                       end else begin
                           b = (r - g)/2;
                       end
                       red_o = b;
                       blue_o = b;
                       green_o = b;
                          red_o = red_o/16;
                          blue_o = blue_o/16;
                          green_o = green_o/16;
                          red = {red_o[3],red_o[2], red_o[1], red_o[0]};
                          green = {green_o[3],green_o[2], green_o[1], green_o[0]};
                          blue = {blue_o[3],blue_o[2], blue_o[1], blue_o[0]};
                        
                    if(addra <18239)
                     addra = addra + 1;
                    else
                    addra = 0; 
                    
                    if(addrb_2 <18239)
                     addrb_2 = addrb_2 + 1;
                    else
                    addrb_2 = 0;             
                    
                    
                    
                    
                    end
                    
                    else if(select == 'b100) begin
                    
                       r = (gray + left + right + up +down +leftup +leftdown +rightup +rightdown);
                       r = r/18;
                       
                       red_o = r;
                       blue_o = r;
                       green_o = r;
                       
                       red_o = red_o/16;
                       blue_o = blue_o/16;
                       green_o = green_o/16;
                       
                       red = {red_o[3],red_o[2], red_o[1], red_o[0]};
                       green = {green_o[3],green_o[2], green_o[1], green_o[0]};
                       blue = {blue_o[3],blue_o[2], blue_o[1], blue_o[0]};  
                       
                     if(addra <18239)
                     addra = addra + 1;
                    else
                    addra = 0;  
                    
                    
                    end
                    
                    else  begin
                    red ='d0;
                    blue = 0;
                    green =0;
                    
                    
                    end
                    
                    
                    
                    
                    end
                    
                    
                    
                    
            else if(blank == 0 && hc >= 479 && hc < 639 && vc >= 0 && vc < 114)
            begin
              
                gray =  {out2[95], out2[94], out2[93], out2[92], out2[91], out2[90], out2[89], out2[88]};
                left = {out2[87], out2[86], out2[85], out2[84], out2[83], out2[82], out2[81], out2[80]};
                right = {out2[79], out2[78], out2[77], out2[76], out2[75], out2[74], out2[73], out2[72]};                
                            
                up =  {out2[71], out2[70], out2[69], out2[68], out2[67], out2[66], out2[65], out2[64]};
                down = {out2[63], out2[62], out2[61], out2[60], out2[59], out2[58], out2[57], out2[56]};
                leftup = {out2[55], out2[54], out2[53], out2[52], out2[51], out2[50], out2[49], out2[48]};
                leftdown =  {out2[47], out2[46], out2[45], out2[44], out2[43], out2[42], out2[41], out2[40]};
                rightup = {out2[39], out2[38], out2[37], out2[36], out2[35], out2[34], out2[33], out2[32]};
                rightdown = {out2[31], out2[30], out2[29], out2[28], out2[27], out2[26], out2[25], out2[24]};
                tblue =  {out2[23], out2[22], out2[21], out2[20], out2[19], out2[18], out2[17], out2[16]};
                tgreen = {out2[15], out2[14], out2[13], out2[12], out2[11], out2[10], out2[9], out2[8]};
                tred = {out2[7], out2[6], out2[5], out2[4], out2[3], out2[2], out2[1], out2[0]};
                
                
                if(hc == 'd479 &&  vc == 'd0) begin
                    addra = 0;
                    
                end
                
                

//                 RGB image to gray scale image
              
                    if(reset) begin
                        red = 0;
                        green = 0;
                        blue = 0;
                        
                    end 
                    
                    else if (select == 'b001) begin
                    
                      red_o = tred/16;
                      blue_o = tblue/16;
                     
                     green_o = tgreen/16;
                    red = {red_o[3],red_o[2], red_o[1], red_o[0]};
                    green = {green_o[3],green_o[2], green_o[1], green_o[0]};
                    blue = {blue_o[3],blue_o[2], blue_o[1], blue_o[0]};  
                        
                    if(addra <18239)
                     addra = addra + 1;
                    else
                    addra = 0;             
                    
                    
                    
                    
                    end
                    
                    else if (select == 'b101)  begin
                       r = (gray + left + right + up +down +leftup +leftdown +rightup +rightdown);
                       r = r/18;
                       
                       red_o = r;
                       blue_o = r;
                       green_o = r;
                       
                       red_o = red_o/16;
                       blue_o = blue_o/16;
                       green_o = green_o/16;
                       
                       red = {red_o[3],red_o[2], red_o[1], red_o[0]};
                       green = {green_o[3],green_o[2], green_o[1], green_o[0]};
                       blue = {blue_o[3],blue_o[2], blue_o[1], blue_o[0]};  
                       
                         if(addra <18239)
                         addra = addra + 1;
                        else
                        addra = 0;  
                    
                    
                    end
                    
                    
                    
                    
                    end
                    
                    
                    
                else if(blank == 0 && hc >= 0 && hc < 160 && vc >= 365 && vc < 479)
                begin
              
                gray =  {out2[95], out2[94], out2[93], out2[92], out2[91], out2[90], out2[89], out2[88]};
                left = {out2[87], out2[86], out2[85], out2[84], out2[83], out2[82], out2[81], out2[80]};
                right = {out2[79], out2[78], out2[77], out2[76], out2[75], out2[74], out2[73], out2[72]};                
                            
                up =  {out2[71], out2[70], out2[69], out2[68], out2[67], out2[66], out2[65], out2[64]};
                down = {out2[63], out2[62], out2[61], out2[60], out2[59], out2[58], out2[57], out2[56]};
                leftup = {out2[55], out2[54], out2[53], out2[52], out2[51], out2[50], out2[49], out2[48]};
                leftdown =  {out2[47], out2[46], out2[45], out2[44], out2[43], out2[42], out2[41], out2[40]};
                rightup = {out2[39], out2[38], out2[37], out2[36], out2[35], out2[34], out2[33], out2[32]};
                rightdown = {out2[31], out2[30], out2[29], out2[28], out2[27], out2[26], out2[25], out2[24]};
                tblue =  {out2[23], out2[22], out2[21], out2[20], out2[19], out2[18], out2[17], out2[16]};
                tgreen = {out2[15], out2[14], out2[13], out2[12], out2[11], out2[10], out2[9], out2[8]};
                tred = {out2[7], out2[6], out2[5], out2[4], out2[3], out2[2], out2[1], out2[0]};
                
                
               if(hc == 'd0 &&  vc == 'd365) begin
                    addra = 0;
                    
                end
                
                

//                 RGB image to gray scale image
              
                    if(reset) begin
                        red = 0;
                        green = 0;
                        blue = 0;
                        
                    end 
                    
                    else if (select == 'b010) begin
                    
                      red_o = tred/16;
                      blue_o = tblue/16;
                     
                     green_o = tgreen/16;
                    red = {red_o[3],red_o[2], red_o[1], red_o[0]};
                    green = {green_o[3],green_o[2], green_o[1], green_o[0]};
                    blue = {blue_o[3],blue_o[2], blue_o[1], blue_o[0]};  
                        
                    if(addra <18239)
                     addra = addra + 1;
                    else
                    addra = 0;             
                    
                    
                    
                    
                    end
                    
                    else if (select == 'b110)  begin
                       r = (gray + left + right + up +down +leftup +leftdown +rightup +rightdown);
                       r = r/18;
                       
                       red_o = r;
                       blue_o = r;
                       green_o = r;
                       
                       red_o = red_o/16;
                       blue_o = blue_o/16;
                       green_o = green_o/16;
                       
                       red = {red_o[3],red_o[2], red_o[1], red_o[0]};
                       green = {green_o[3],green_o[2], green_o[1], green_o[0]};
                       blue = {blue_o[3],blue_o[2], blue_o[1], blue_o[0]};  
                       
                     if(addra <18239)
                     addra = addra + 1;
                    else
                    addra = 0;  
                    
                    
                    end
                    
                    
                    
                    
                    end
                    
                    
                                        
                else if(blank == 0 && hc >= 479 && hc < 639 && vc >= 365 && vc < 479)
                begin
              
                gray =  {out2[95], out2[94], out2[93], out2[92], out2[91], out2[90], out2[89], out2[88]};
                left = {out2[87], out2[86], out2[85], out2[84], out2[83], out2[82], out2[81], out2[80]};
                right = {out2[79], out2[78], out2[77], out2[76], out2[75], out2[74], out2[73], out2[72]};                
                            
                up =  {out2[71], out2[70], out2[69], out2[68], out2[67], out2[66], out2[65], out2[64]};
                down = {out2[63], out2[62], out2[61], out2[60], out2[59], out2[58], out2[57], out2[56]};
                leftup = {out2[55], out2[54], out2[53], out2[52], out2[51], out2[50], out2[49], out2[48]};
                leftdown =  {out2[47], out2[46], out2[45], out2[44], out2[43], out2[42], out2[41], out2[40]};
                rightup = {out2[39], out2[38], out2[37], out2[36], out2[35], out2[34], out2[33], out2[32]};
                rightdown = {out2[31], out2[30], out2[29], out2[28], out2[27], out2[26], out2[25], out2[24]};
                tblue =  {out2[23], out2[22], out2[21], out2[20], out2[19], out2[18], out2[17], out2[16]};
                tgreen = {out2[15], out2[14], out2[13], out2[12], out2[11], out2[10], out2[9], out2[8]};
                tred = {out2[7], out2[6], out2[5], out2[4], out2[3], out2[2], out2[1], out2[0]};

                
                
                if(hc == 'd479 &&  vc == 'd365) begin
                    addra = 0;
                    
                end
                

//                 RGB image to gray scale image
              
                    if(reset) begin
                    
                        red = 0;
                        green = 0;
                        blue = 0;
                        
                    end 
                    
                    else if (select == 'b011) begin
                    
                      red_o = tred/16;
                      blue_o = tblue/16;
                     
                     green_o = tgreen/16;
                    red = {red_o[3],red_o[2], red_o[1], red_o[0]};
                    green = {green_o[3],green_o[2], green_o[1], green_o[0]};
                    blue = {blue_o[3],blue_o[2], blue_o[1], blue_o[0]};  
                        
                    if(addra <18239)
                     addra = addra + 1;
                    else
                    addra = 0;             
                    
                    
                    
                    
                    end
                    
                    else if (select == 'b111)  begin
                       r = (gray + left + right + up +down +leftup +leftdown +rightup +rightdown);
                       r = r/18;
                       
                       red_o = r;
                       blue_o = r;
                       green_o = r;
                       
                       red_o = red_o/16;
                       blue_o = blue_o/16;
                       green_o = green_o/16;
                       
                       red = {red_o[3],red_o[2], red_o[1], red_o[0]};
                       green = {green_o[3],green_o[2], green_o[1], green_o[0]};
                       blue = {blue_o[3],blue_o[2], blue_o[1], blue_o[0]};  
                       
                     if(addra <18239)
                     addra = addra + 1;
                    else
                    addra = 0;  
                    
                    
                    end
                    
                    
                    
                    
                    end
                    
                    
                    
                    
                    else 
                    begin
                    red =0;
                    blue=0;
                    green =0;
                    end
                    
                    
                    
                     if(temp_sel ^ select) 
                        begin
                            addra = 0;
                            temp_sel = select;
                        end
                    
                    
                    
            


          end
        end 
       
    endmodule
