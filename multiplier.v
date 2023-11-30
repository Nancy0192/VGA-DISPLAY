
module multiplier
(
	input  [7:0] in1,
	input  [7:0] in2,
	input start,
	input  reset,
	input  clk,

	output reg [15:0] out,
	output reg done
);



parameter IDLE = 2'b00;
parameter RUN = 2'b01;
parameter DONE = 2'b10;

reg [1:0] state, next_state;

reg load,clear;
reg [15:0] partial,par_out_a;
reg [7:0] par_out_b;
reg [3:0] count;


always @ (posedge clk, posedge reset)				//SHIFT REGISTER A 32 bit
begin 
	if (reset == 1'b1)
		par_out_a <= 16'd0;
	else if(load == 1'b1)
		par_out_a <= {8'd0,in1}; 	
	else	
		par_out_a <= {par_out_a[14:0],1'b0};
				
end

always @ (posedge clk, posedge reset)				//SHIFT REGISTER B 16 bit
begin
	if (reset == 1'b1)
		par_out_b <= 8'd0;
	else if(load == 1'b1)
		par_out_b <= in2; 	
	else	
		par_out_b <= {1'b0, par_out_b[7:1]};
				
end

always @(par_out_b,par_out_a)					//MULTIPLEXER
begin
	if(par_out_b[0] == 1'b0)
		partial <= 16'd0;
	else
		partial <= par_out_a;
end

always @ (posedge clk, posedge reset)				//ACCUMULATOR
begin
	if (reset == 1'b1 || clear)
		out <= 16'd0;
	else
		out <= out+partial;
end


always@ (posedge clk, posedge reset)				//COUNTER
begin
	if (reset == 1'b1 || state!=RUN)
		count <= 4'b1010;
	else 
		count <= count-4'b0001;
end

always @ (posedge clk, posedge reset)				//STATE REGISTER
begin
	if (reset == 1'b1)
		state <= IDLE;
	else
		state <= next_state;
end


always@(start,count,state)					//FSM COMB
begin
	case(state)
		IDLE:
		begin
			if(start == 1)
				next_state <= RUN;
			else
				next_state <= IDLE;
			
			load <= 0;
			clear <= 1;
			done <= 0;
		end
		
		RUN:
		begin
			if(count == 4'b0000)
				next_state <= DONE;
			else
				next_state <= RUN;
			
			if(count == 4'b1001) 
				load <= 1;
			else
				load <= 0;
			clear <= 0;
			done <= 0;
		end

		DONE:
		begin
			next_state <= IDLE;
			load <= 0;
			clear <= 0;
			done <= 1;
		end
		
		default:
		begin
			next_state <= IDLE;
			load <= 0;
			clear <= 1;
			done <= 0;
		end
		
	endcase
end
	
endmodule


