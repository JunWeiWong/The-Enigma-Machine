module display
	(
		clock,						//	On Board 50 MHz
		// Your inputs and outputs here
        in,
        press,
		  reset,
		// The ports below are for the VGA output.  Do not change.
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,						//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B   						//	VGA Blue[9:0]
	);

	input			clock;				//	50 MHz
	input[4:0] in;
	input press, reset;

	// Declare your inputs and outputs here
	// Do not change the following outputs
	output			VGA_CLK;   				//	VGA Clock
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;				//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[9:0]	VGA_R;   				//	VGA Red[9:0]
	output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
	output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
	
	
	// Create the colour, x, y and writeEn wires that are inputs to the controller.
	wire [2:0] colour;
	wire [7:0] x;
	wire [6:0] y;
	wire writeEn, draw, erase;

	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	vga_adapter VGA(
			.resetn(~reset),
			.clock(clock),
			.colour(colour),
			.x(x),
			.y(y),
			.plot(writeEn),
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
		defparam VGA.BACKGROUND_IMAGE = "image.colour.mif";
			
	// Put your code here. Your code should produce signals x,y,colour and writeEn/plot
	// for the VGA controller, in addition to any other functionality your design may require.
    
    // Instansiate datapath
	// datapath d0(...);


		datapath d0(
			.in(in),
			.draw(draw),
			.erase(erase),
			.clk(clock),
			.out_c(colour),
			.out_x(x),
			.out_y(y));

    // Instansiate FSM control
    // control c0(...);

		control c0(
			.clk(clock),
			.press(press),
			.reset(reset),
			.draw(draw),
			.erase(erase),
			.writeEn(writeEn)
			);
    
endmodule

//module TestModule(CLOCK_50, SW, KEY, out_c, out_x, out_y, writeEn);
//	input	CLOCK_50;
//	input [9:0] SW;
//	input [3:0] KEY;
//	output [2:0] out_c;
//	output [7:0] out_x;
//	output [6:0] out_y;
//	output writeEn;
//	wire loadx, loady, loadc, enable;
//
//	assign start = ~KEY[1];
//	assign load = ~KEY[3];
//
//	control c0(start, load, KEY[0], CLOCK_50, loadx, loady, loadc, enable, writeEn);
//
//
//	datapath d0(SW[6:0], SW[9:7],loadx,loady,loadc,enable,CLOCK_50,KEY[0],out_c,out_x,out_y);
//endmodule

module control(clk, press, reset, draw, erase, writeEn);
    input press, clk, reset;
    output reg draw, erase, writeEn;

    reg [3:0] current_state, next_state; 
    localparam  erasing    = 3'd0,
                drawing   = 3'd1,
                waiting    = 3'd2;

    always @(posedge clk)
    begin: state_table 
            case (current_state)
                waiting: next_state = press ? erasing : waiting; 
//                sload_xw: next_state = load ? sload_xw : sload_y; 
                erasing: next_state = drawing;
					 drawing: next_state = waiting;
//                sload_yw: next_state = start ? sload_yw : sdraw;
//                sdraw: next_state = load? sdraw : sload_x;
					 default:     next_state = waiting;
				endcase
    end

    always @(*) 
    begin: enable_signals
        erase = 1'b0;
        draw = 1'b0;
		  writeEn = 1'b0;
  
        case (current_state)
            erasing: begin
                erase = 1'b1;
            end
            drawing: begin
                draw = 1'b1;
					 writeEn = 1'b1;
            end
        endcase
    end

    always @(posedge clk)
    begin
        if (reset == 1'b1) 
            begin
                current_state <= waiting;
            end
        else 
            begin
                current_state <= next_state;
            end
    end
endmodule



module datapath(in,draw,erase,clk,out_c,out_x,out_y);
	input [4:0] in;
	input draw, erase, clk;
 
	output reg[7:0] out_x;
	output reg[6:0] out_y;
	output reg[2:0] out_c;
	reg [7:0] pre_x;
	wire [7:0] cur_x;
	reg [6:0] pre_y;
	wire [6:0] cur_y;
//	reg [7:0] ind_x;
//	reg [6:0] ind_y;
	
	indicator i(.in(in), .x(cur_x), .y(cur_y));
	
//	initial begin
//		pre_x <= 8'b0;
//		pre_y <= 7'b0;
//	end
	
	always @(posedge clk)
		begin
			if (erase == 1'b1)
				begin
					out_x <= pre_x;
					out_y <= pre_y;
					out_c <= 3'b000;
				end
			else if (draw == 1'b1)
				begin
					out_x <= cur_x;
					out_y <= cur_y;
					out_c <= 3'b111;
					pre_x <= cur_x;
					pre_y <= cur_y;
				end
		end
endmodule

module indicator(in, x, y);
	input[4:0] in;
	output reg[7:0] x;
	output reg[6:0] y;
	
	always @(*)
	begin
		case (in)
			5'b00000: begin
				x <= 8'd 24;
				y <= 7'd 64;
			end
			5'b00001: begin
				x <= 8'd 89;
				y <= 7'd 90;
			end
			5'b00010: begin
				x <= 8'd 60;
				y <= 7'd 90;
			end
			5'b00011: begin
				x <= 8'd 53;
				y <= 7'd 64;
			end
			5'b00100: begin
				x <= 8'd 51;
				y <= 7'd 40;
			end
			5'b00101: begin
				x <= 8'd 66;
				y <= 7'd 64;
			end
			5'b00110: begin
				x <= 8'd 81;
				y <= 7'd 64;
			end
			5'b00111: begin
				x <= 8'd 95;
				y <= 7'd 64;
			end
			5'b01000: begin
				x <= 8'd 117;
				y <= 7'd 40;
			end
			5'b01001: begin
				x <= 8'd 108;
				y <= 7'd 64;
			end
			5'b01010: begin
				x <= 8'd 122;
				y <= 7'd 64;
			end
			5'b01011: begin
				x <= 8'd 135;
				y <= 7'd 64;
			end
			5'b01100: begin
				x <= 8'd 121;
				y <= 7'd 90;
			end
			5'b01101: begin
				x <= 8'd 104;
				y <= 7'd 90;
			end
			5'b01110: begin
				x <= 8'd 130;
				y <= 7'd 40;
			end
			5'b01111: begin
				x <= 8'd 145;
				y <= 7'd 40;
			end
			5'b10000: begin
				x <= 8'd 17;
				y <= 7'd 40;
			end
			5'b10001: begin
				x <= 8'd 65;
				y <= 7'd 40;
			end
			5'b10010: begin
				x <= 8'd 39;
				y <= 7'd 64;
			end
			5'b10011: begin
				x <= 8'd 77;
				y <= 7'd 40;
			end
			5'b10100: begin
				x <= 8'd 104;
				y <= 7'd 40;
			end
			5'b10101: begin
				x <= 8'd 75;
				y <= 7'd 90;
			end
			5'b10110: begin
				x <= 8'd 34;
				y <= 7'd 40;
			end
			5'b10111: begin
				x <= 8'd 45;
				y <= 7'd 90;
			end
			5'b11000: begin
				x <= 8'd 90;
				y <= 7'd 40;
			end
			5'b11001: begin
				x <= 8'd 33;
				y <= 7'd 90;
			end
			default: begin
				x <= 200;
				y <= 200;
			end
		endcase
	end	
	
endmodule