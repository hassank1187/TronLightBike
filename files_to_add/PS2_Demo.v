
module ps2c (
	// Inputs
	CLOCK_50,
	KEY,

	// Bidirectionals
	PS2_CLK,
	PS2_DAT,
	
	// Outputs
//	HEX0,
//	HEX1,
//	HEX2,
//	HEX3,
//	HEX4,
//	HEX5,
//	HEX6,
//	HEX7,
	//LEDR,
	PLAYERONE,
	PLAYERTWO
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/


/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/

// Inputs
input				CLOCK_50;
input		[3:0]	KEY;

// Bidirectionals
inout				PS2_CLK;
inout				PS2_DAT;

// Outputs
//output		[6:0]	HEX0;
//output		[6:0]	HEX1;
//output		[6:0]	HEX2;
//output		[6:0]	HEX3;
//output		[6:0]	HEX4;
//output		[6:0]	HEX5;
//output		[6:0]	HEX6;
//output		[6:0]	HEX7;
//output		[9:0]	LEDR;
output 		[3:0] PLAYERONE;
output 		[3:0] PLAYERTWO;
/*****************************************************************************
 *                 Internal Wires and Registers Declarations                 *
 *****************************************************************************/

// Internal Wires
wire		[7:0]	ps2_key_data;
wire				ps2_key_pressed;

// Internal Registers
reg			[7:0]	last_data_received;

// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/


/*****************************************************************************
 *                             Sequential Logic                              *
 *****************************************************************************/

always @(posedge CLOCK_50)
begin
	if (KEY[0] == 1'b0)
		last_data_received <= 8'h00;
	else if (ps2_key_pressed == 1'b1)
		last_data_received <= ps2_key_data;
end

/*****************************************************************************
 *                            Combinational Logic                            *
 *****************************************************************************/

//assign HEX2 = 7'h7F;
//assign HEX3 = 7'h7F;
//assign HEX4 = 7'h7F;
//assign HEX5 = 7'h7F;
//assign HEX6 = 7'h7F;
//assign HEX7 = 7'h7F;

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

PS2_Controller PS2 (
	// Inputs
	.CLOCK_50			(CLOCK_50),
	.reset				(~KEY[0]),

	// Bidirectionals
	.PS2_CLK			(PS2_CLK),
 	.PS2_DAT			(PS2_DAT),

	// Outputs
	.received_data		(ps2_key_data),
	.received_data_en	(ps2_key_pressed)
);

//Hexadecimal_To_Seven_Segment Segment0 (
//	// Inputs
//	.hex_number			(last_data_received[3:0]),
//
//	// Bidirectional
//
//	// Outputs
//	.seven_seg_display	(HEX0)
//);
//
//Hexadecimal_To_Seven_Segment Segment1 (
//	// Inputs
//	.hex_number			(last_data_received[7:4]),
//
//	// Bidirectional
//
//	// Outputs
//	.seven_seg_display	(HEX1)
//);

ps2tocontrol control(
.last_data_received(last_data_received[7:0]),
.resetn(KEY[0]),
.clk(CLOCK_50),
.playerone(PLAYERONE[3:0]),
.playertwo(PLAYERTWO[3:0])
);
endmodule

module ps2tocontrol (
last_data_received,
clk,
resetn, playerone, playertwo);

input [7:0] last_data_received;
input resetn;
input clk;
output reg [3:0] playerone;
output reg [3:0] playertwo;


always@(posedge clk)


begin
	if(!resetn)
	begin
		playerone <= 4'b0;
		playertwo <= 4'b0;
	end
	else
	begin
		case(last_data_received)
		8'h1D: playerone 		<= 4'b1000; //w
		8'h1C: playerone 		<= 4'b0100; //a
		8'h1B: playerone 		<= 4'b0010; //s
		8'h23: playerone 		<= 4'b0001; //d
		8'h43: playertwo 		<= 4'b1000; //i
		8'h3B: playertwo 		<= 4'b0100; //j
		8'h42: playertwo 		<= 4'b0010; //k
		8'h4B: playertwo 		<= 4'b0001; //l
		default: begin
		playerone <= 4'b0;
		playertwo <= 4'b0;
				end
		endcase
	end
end

endmodule
