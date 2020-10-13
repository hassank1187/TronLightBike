module finalproject
	(
		CLOCK_50,						//	On Board 50 MHz
		// Your inputs and outputs here
		SW,
		KEY,							// On Board Keys
		// The ports below are for the VGA output.  Do not change.
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,						//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,   						//	VGA Blue[9:0]
		//
		PS2_CLK,
		PS2_DAT,
		HEX0,
//		HEX1,
		HEX2,
//		HEX3,
//		HEX4,
//		HEX5,
//		HEX6,
//		HEX7,
		LEDR
	);
	
	 input [9:0]SW;
	 input [9:0]KEY;
	 output [6:0]HEX0;
//	 output [6:0]HEX1;
	 output [6:0]HEX2;
//	 output [6:0]HEX3;
//	 output [6:0]HEX4;
//	 output [6:0]HEX5;
//	 output [6:0]HEX6;
//	 output [6:0]HEX7;
	 output [9:0]LEDR;
	 inout	   PS2_CLK;
	 inout	   PS2_DAT;
	 input		CLOCK_50;//	50 MHz
	 
	 wire
	 dir1,
	 move1,
	 draw1,
	 dir2,
	 move2,
	 draw2;
	 
	 wire write, read;
	 wire [2:0] win;
	 
	 wire draw = draw1 | draw2;
	 
	 wire [7:0] x;
	 wire [7:0] y;
	 wire [2:0] colour;
	 
				
	// Declare your inputs and outputs here
	// Do not change the following outputs
	output			VGA_CLK;   				//	VGA Clock
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;				//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[7:0]	VGA_R;   				//	VGA Red[7:0] Changed from 10 to 8-bit DAC
	output	[7:0]	VGA_G;	 				//	VGA Green[7:0]
	output	[7:0]	VGA_B;   				//	VGA Blue[7:0]
	
	
	// Create the colour, x, y and writeEn wires that are inputs to the controller.
	

	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	vga_adapter VGA(
			.resetn(SW[9]),
			.clock(CLOCK_50),
			.colour(colour),
			.x(x),
			.y(y),
			.plot(draw),
			/* Signals for the DAC to drive the monitor. */
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "black.mif";
	
wire [3:0] control1;
wire [3:0] control2;
	ps2c controller(
	// Inputs
	.CLOCK_50(CLOCK_50),
	.KEY(KEY[3:0]),

	// Bidirectionals
	.PS2_CLK(PS2_CLK),
	.PS2_DAT(PS2_DAT),
	
	// Outputs
//	.HEX0(HEX0[6:0]),
//	.HEX1(HEX1[6:0]),
//	.HEX2(HEX2[6:0]),
//	.HEX3(HEX3[6:0]),
//	.HEX4(HEX4[6:0]),
//	.HEX5(HEX5[6:0]),
//	.HEX6(HEX6[6:0]),
//	.HEX7(HEX7[6:0]),
	//.LEDR(LEDR),
	.PLAYERONE(control1[3:0]),
	.PLAYERTWO(control2[3:0])
);

ram64kx3 collision(
	.address({x[7:0],y[7:0]}),
	.clock(CLOCK_50),
	.data(colour),
	.rden(read),
	.wren(write),
	.q(win));

	
	
	control c1(
    .clk(CLOCK_50),
    .resetn(SW[9]),
	 .win(win),
	 
    .e_dir1(dir1),
	 .e_move1(move1),
	 .e_draw1(draw1),
	 
    .p1end(LEDR[1]),
	 .p2end(LEDR[0]),
	 .tie_end(LEDR[2]),
	  
	 .clr1(LEDR[9:7]),
	 .clr2(LEDR[5:3]),
	 
	 .e_dir2(dir2),
	 .e_move2(move2),
    .e_draw2(draw2),
	 .e_write(write),
	 .e_read(read)
    );
	 
	 datapath c2(
    .clk(CLOCK_50),
    .resetn(SW[9]),
	 
	 .w(control1[3]),
	 .a(control1[2]),
	 .s(control1[1]),
	 .d(control1[0]),
	 
	 .i(control2[3]),
	 .j(control2[2]),
	 .k(control2[1]),
	 .l(control2[0]),
	 
	 
	 .e_dir1(dir1),
	 .e_move1(move1),
	 .e_draw1(draw1),
		 
	 .e_dir2(dir2),
	 .e_move2(move2),
	 .e_draw2(draw2),
    .x(x),
	 .y(y),
	 .colour(colour)
    );
	
	Hexadecimal_To_Seven_Segment Segment2 (
	// Inputs
	.hex_number			(colour),

	// Bidirectional

	// Outputs
	.seven_seg_display	(HEX2)
);

Hexadecimal_To_Seven_Segment Segment0 (
	// Inputs
	.hex_number			(win),

	// Bidirectional

	// Outputs
	.seven_seg_display	(HEX0)
);

endmodule
  
                
module control(
	  clk,
	  resetn,
	  win,
	 
	  clr1,
	  clr2,
	  
	  e_dir1,
	  e_move1,
	  e_draw1,
		 
	  e_dir2,
	  e_move2,
	  e_draw2,
	  
	  p1end,
	  p2end,
	  tie_end,
		 
	  e_write,
	  e_read
	  
    );
	 input 
	 clk, 
	 resetn, 
	 win;
	 
	 output reg [2:0] 
	 clr1, 
	 clr2;
	  
	 output reg
	 e_dir1,
	 e_move1,
	 e_draw1,
		 
	 e_dir2,
	 e_move2,
	 e_draw2,
	  
	 p1end,
	 p2end,
	 tie_end,
		 
	 e_write,
	 e_read;
	 
    reg [5:0] current_state, next_state;
    reg P1win, P2win;
    
    localparam  IDLE        = 5'd0,
					 DIR1			 = 5'd1,
                MOVE1       = 5'd2,
                DRAW1	    = 5'd3,
					 
				    DIR2			 = 5'd4,
					 MOVE2		 = 5'd5,
					 DRAW2		 = 5'd6,
                WAIT        = 5'd7,
					 
					 READ1 		 = 5'd8,
					 WRITE1		 = 5'd9,
					 READ2		 = 5'd10,
					 WRITE2		 = 5'd11,
					 
					 P1			 = 5'd13,
					 P2			 = 5'd14,
					 TIE			 = 5'd15;
					 
    
	 reg [19:0]counter;
	 reg [27:0]startcounter;
    // Next state logic aka our state table
    always@(*)
    begin: state_table
            case (current_state)
                IDLE: next_state = (startcounter== 27'b0)?DIR1:IDLE;
					 
					 DIR1: next_state = MOVE1;
					 MOVE1: next_state = READ1;
					 READ1: next_state = DRAW1;
					 DRAW1: next_state = WRITE1;
					 WRITE1: next_state = DIR2;
					 
					 DIR2: next_state = MOVE2;
					 MOVE2: next_state = READ2;
					 READ2: next_state = DRAW2;
					 DRAW2: next_state = WRITE2;
					 WRITE2: next_state = WAIT;
					 
//					 P1: next_state = (P1win && !P2win)? P1: P2;
//					 P2: next_state = (!P1win && P2win)? P2: TIE;
//					 TIE: next_state = (P1win && P2win)? TIE : WAIT;
					 
					 
                WAIT: next_state=(counter == 20'b0)? DIR1:WAIT;
					 
            default:     next_state = IDLE;
        endcase
    end // state_table
   
	
	
always@(posedge clk)
begin
					if(!resetn) counter<=20'b11111111111111111111;
					else if(current_state == WAIT)  counter<=counter-1'b1;
					else counter <=20'b11111111111111111111;
end

always@(posedge clk)
begin
					if(!resetn) startcounter<=28'b1101111101011110000100000000;
					else if(current_state == IDLE)  startcounter<=startcounter-1'b1;
					else startcounter <=28'b1101111101011110000100000000;
end


always @(posedge clk)
begin
	if(!resetn)
		begin
			clr1 <= 3'b0;
			clr2 <= 3'b0;
		end
	else
	begin
		if(current_state == READ1)
					clr1 <= win;
		  
		if(current_state == READ2)
					clr2 <= win;
	end
		  
end

always @(posedge clk)
begin

	if(!resetn)
	begin
		P1win <= 1'b0;
		P2win <= 1'b0;
	end
	else
		begin
		if(clr1 != 3'b0)
			P1win <= 1'b1;
		if(clr2 != 3'b0)
			P2win <= 1'b1;
		end
end

always @(*)
begin: enable_signals
		 e_dir1 = 1'b0;
		 e_move1= 1'b0;
		 e_draw1= 1'b0;
		 
		 e_dir2 =1'b0;
		 e_move2=1'b0;
		 e_draw2=1'b0;
		 
		 e_write = 1'b0;
		 e_read  = 1'b0;
		 
		 p1end = 1'b0;
		 p2end = 1'b0;
		 tie_end = 1'b0;
		 
        case (current_state)
		  DIR1: e_dir1=1'b1;
		  MOVE1: e_move1=1'b1;
		  DRAW1: e_draw1=1'b1;
		  
		  DIR2: e_dir2=1'b1;
		  MOVE2: e_move2=1'b1;
		  DRAW2: e_draw2=1'b1;
		  
		  READ1:e_read = 1'b1;
		  READ2:e_read = 1'b1;
		  
		  WRITE1:e_write=1'b1;
		  WRITE2:e_write=1'b1;
		  
		  P1:  p1end=1'b1;
		  P2:  p2end=1'b1;
		  TIE: tie_end=1'b1;
		  
        // default:    // don't need default since we already made sure all of our outputs were assigned a value at the start of the always block
        endcase
end // enable_signals

always@(posedge clk)
begin
        if(!resetn)
            current_state <= IDLE;
        else
            current_state <= next_state;
end
endmodule



module datapath(
    input clk,
    input resetn,
	 input w,a,s,d,i,j,k,l,
	 
	 
	 input 
	 e_dir1,
	 e_move1,
	 e_draw1,
		 
	 e_dir2,
	 e_move2,
	 e_draw2,
	 
    output reg [7:0] x,
	 output reg [7:0] y,
	 output reg [2:0] colour
    );
    //directions
	 localparam NORTH = 3'b001,
					EAST =  3'b010,
					SOUTH = 3'b011,
					WEST =  3'b100;

    reg [7:0] x1, x2; 
	 reg [7:0] y1, y2;
	 reg [2:0] dir1, dir2;
    
always@(posedge clk)
begin
		  if(!resetn)
				begin
					dir1 <= EAST;
				end
	     else
	     
		  if(e_dir1)
		  begin
		  if(dir1 == NORTH && a)
				dir1 = WEST;
		  if(dir1 == NORTH && d)
				dir1 = EAST;
		  
		  if(dir1 == EAST && w)
				dir1 = NORTH;
		  if(dir1 == EAST & s)
				dir1 = SOUTH;
				
		  if(dir1 == SOUTH && a)
				dir1 = WEST;
		  if(dir1 == SOUTH && d)
				dir1 = EAST;
				
		  if(dir1 == WEST && w)
				dir1 = NORTH;
		  if(dir1 == WEST && s)
				dir1 = SOUTH;
		  end
end

always@(posedge clk)
begin
				if(!resetn)
				begin
					dir2 <= WEST;
				end
				else
				
				if(e_dir2)
		  begin
		  if(dir2 == NORTH && j)
				dir2 = WEST;
		  if(dir2 == NORTH && l)
				dir2 = EAST;
		  
		  if(dir2 == EAST && i)
				dir2 = NORTH;
		  if(dir2 == EAST & k)
				dir2 = SOUTH;
				
		  if(dir2 == SOUTH && j)
				dir2 = WEST;
		  if(dir2 == SOUTH && l)
				dir2 = EAST;
				
		  if(dir2 == WEST && i)
				dir2 = NORTH;
		  if(dir2 == WEST && k)
				dir2 = SOUTH;
		  end
end

always@(posedge clk) 
begin
        if(!resetn) 
			begin
            x1 <= 8'b00000001; 
            y1 <= 8'b00111011;
            x2 <= 8'b10011111; 
            y2 <= 8'b00111011;
			end
        else 
		  
	begin
	if(e_move1 == 1'b1)
			begin
				if(dir1 == WEST)
                begin
						x1 <= x1-1'b1;
					 end
            if(dir1 == EAST)
                begin
						x1 <= x1+1'b1;
					 end
            if(dir1 == NORTH)
                begin
						y1 <= y1-1'b1;
					 end
            if(dir1 == SOUTH)
                begin
						y1 <= y1+1'b1;
					 end
				 if(x1 == 8'b10100001)
					 x1 <= 8'b0;
				 if(x1 == 8'b11111111)
					 x1 <= 8'b10100000;
				 if(y1 == 8'b01111001)
					 y1 <= 8'b0;
				 if(y1 == 8'b11111111)
					 y1 <= 8'b1111000;
			end
			else
			begin
			x1 <= x1;
			y1 <= y1;
			end
			
	if(e_move2 == 1'b1)
			begin
				if(dir2 == WEST)
                begin
						x2 <= x2-1'b1;
					 end
            if(dir2 == EAST)
                begin
						x2 <= x2+1'b1;
					 end
            if(dir2 == NORTH)
                begin
						y2 <= y2-1'b1;
					 end
            if(dir2 == SOUTH)
                begin
						y2 <= y2+1'b1;
					 end
				 if(x2 == 8'b10100001)
					 x2 <= 8'b0;
				 if(x2 == 8'b11111111)
					 x2 <= 8'b10100000;
				 if(y2 == 8'b01111001)
					 y2 <= 8'b0;
				 if(y2 == 8'b11111111)
					 y2 <= 8'b1111000;
			end
		else
			begin
			x2 <= x2;
			y2 <= y2;
			end
	end
end

    // Output result register
 always@(posedge clk) 
 begin
        if(!resetn) begin
            x <= 8'b0;
				y <= 7'b0;
				colour <= 3'b0;
        end
        else
				if(e_move1)
				begin
            x <= x1;
				y <= y1;
				colour <= 3'b100;
				end
				else
				if(e_move2)
				begin
				x <= x2;
				y <= y2;
				colour <= 3'b001;
				end
end
endmodule