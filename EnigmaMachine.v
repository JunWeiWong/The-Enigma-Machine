//`include "rotor.v"
//`include "reflector.v"

module EnigmaMachine(SW, KEY, LEDR, HEX0, HEX1, HEX2, HEX3, CLOCK_50, PS2_DAT, PS2_CLK, VGA_CLK, VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_R, VGA_G, VGA_B);
	 input [9:0] SW;
	 input [3:0] KEY;
	 input CLOCK_50;
	 input PS2_DAT;
	 input PS2_CLK;
	 output[9:0] LEDR;
	 output[6:0] HEX0, HEX1, HEX2, HEX3;
	 output		 VGA_CLK;   				//	VGA Clock
	 output		 VGA_HS;					//	VGA H_SYNC
	 output		 VGA_VS;					//	VGA V_SYNC
	 output		 VGA_BLANK_N;				//	VGA BLANK
 	 output  	 VGA_SYNC_N;				//	VGA SYNC
    output [9:0]VGA_R;   				//	VGA Red[9:0]
	 output [9:0]VGA_G;	 				//	VGA Green[9:0]
	 output [9:0]VGA_B;   				//	VGA Blue[9:0]
	 wire [25:0] key_out, rotor_out, ref_out, rotor_out2, rotor2_out, rotor2_out2, plugboard_in, plugboard_out;
	 wire [4:0] cov_out2;
	 wire press;
	 wire [4:0] state_o, state2_o;
	 //assign LEDR[2] = press;
	 keyboardm k0(.PS2_CLK(PS2_CLK), .PS2_DAT(PS2_DAT), .CLOCK_50(CLOCK_50), .letter(key_out), .ready(press));
	 plugboard p0(.in(key_out), .out(plugboard_in));
	 rotor r0(.in(plugboard_in), .out(rotor_out), .clock(CLOCK_50), .rotate(press), .set(~KEY[0]), .set_state(SW[4:0]), .state(state_o));
	 rotor2 r20(.in(rotor_out), .out(rotor2_out), .clock(CLOCK_50), .rotate(press), .set(~KEY[0]), .set_state(SW[9:5]), .state(state2_o));
	 reflector ref0(.in(rotor2_out), .out(ref_out));
	 rotor2 r20r(.in(ref_out), .out(rotor2_out2), .clock(CLOCK_50), .rotate(press), .set(~KEY[0]), .set_state(SW[9:5]), .state());
	 rotor r0r(.in(rotor2_out2), .out(rotor_out2), .clock(CLOCK_50), .rotate(press), .set(~KEY[0]), .set_state(SW[4:0]), .state());
	 plugboard p1(.in(rotor_out2), .out(plugboard_out));
	 alphabet_to_binary a0(.in(plugboard_out), .out(cov_out2));
	 display d(.clock(CLOCK_50), .in(cov_out2), .press(press), .reset(~KEY[3]),
		// The ports below are for the VGA output.  Do not change.
		.VGA_CLK(VGA_CLK),   						//	VGA Clock
		.VGA_HS(VGA_HS),							//	VGA H_SYNC
		.VGA_VS(VGA_VS),							//	VGA V_SYNC
		.VGA_BLANK_N(VGA_BLANK_N),						//	VGA BLANK
		.VGA_SYNC_N(VGA_SYNC_N),						//	VGA SYNC
		.VGA_R(VGA_R),   						//	VGA Red[9:0]
		.VGA_G(VGA_G),	 						//	VGA Green[9:0]
		.VGA_B(VGA_B));
    
	 MorseCodeDecoder m0(.clock(CLOCK_50), .in(cov_out2), .load(~KEY[2]), .show(~KEY[1]), .reset(KEY[3]), .out(LEDR[0]));
    hex_decoder h0(.hex_digit(state_o[3:0]), .segments(HEX0));
    hex_decoder h1(.hex_digit({3'b000, state_o[4]}), .segments(HEX1));
    hex_decoder h2(.hex_digit(state2_o[3:0]), .segments(HEX2));
    hex_decoder h3(.hex_digit({3'b000, state2_o[4]}), .segments(HEX3));
endmodule

module keyboardm(PS2_CLK, PS2_DAT, CLOCK_50, letter, ready);
	input PS2_DAT;
	input PS2_CLK;
	input CLOCK_50;
	output reg ready;

	// Don't forget to take in PS2_CLK and PS2_DAT as inputs to your top level module.
	// RELEVANT FOR PS2 KB
	wire [7:0] scan_code;
	wire read, scan_ready;
	reg [7:0] scan_history[1:2];
	//	output wire ready;
	
	always @(posedge scan_ready)
	begin
		scan_history [2] <= scan_history[1];
		scan_history [1] <= scan_code;
	end
	
	// END OF PS2 KB SETUP
	// Keyboard Section
	keyboard kb (.keyboard_clk(PS2_CLK ),
		. keyboard_data ( PS2_DAT ),
		. clock50 ( CLOCK_50 ),
		. reset ( 0 ),
		. read ( read ),
		. scan_ready ( scan_ready ),
		. scan_code ( scan_code ));
	
	oneshot pulse (
		. pulse_out ( read ),
		. trigger_in ( scan_ready ),
		. clk ( CLOCK_50 ));
		
	output wire[25:0] letter; 

	assign letter[0] = ((scan_history[1] == 'h1C) && (scan_history[2][7:4] != 'hF)); // Key for A
	assign letter[1] = ((scan_history[1] == 'h32) && (scan_history[2][7:4] != 'hF)); // Key for B
	assign letter[2] = ((scan_history[1] == 'h21) && (scan_history[2][7:4] != 'hF)); // Key for C
	assign letter[3] = ((scan_history[1] == 'h23) && (scan_history[2][7:4] != 'hF)); // Key for D
	assign letter[4] = ((scan_history[1] == 'h24) && (scan_history[2][7:4] != 'hF)); // Key for E
	assign letter[5] = ((scan_history[1] == 'h2B) && (scan_history[2][7:4] != 'hF)); // Key for F
	assign letter[6] = ((scan_history[1] == 'h34) && (scan_history[2][7:4] != 'hF)); // Key for G
	assign letter[7] = ((scan_history[1] == 'h33) && (scan_history[2][7:4] != 'hF)); // Key for H
	assign letter[8] = ((scan_history[1] == 'h43) && (scan_history[2][7:4] != 'hF)); // Key for I
	assign letter[9] = ((scan_history[1] == 'h3B) && (scan_history[2][7:4] != 'hF)); // Key for J
	assign letter[10] = ((scan_history[1] == 'h42) && (scan_history[2][7:4] != 'hF)); // Key for K
	assign letter[11] = ((scan_history[1] == 'h4B) && (scan_history[2][7:4] != 'hF)); // Key for L
	assign letter[12] = ((scan_history[1] == 'h3A) && (scan_history[2][7:4] != 'hF)); // Key for M
	assign letter[13] = ((scan_history[1] == 'h31) && (scan_history[2][7:4] != 'hF)); // Key for N
	assign letter[14] = ((scan_history[1] == 'h44) && (scan_history[2][7:4] != 'hF)); // Key for O
	assign letter[15] = ((scan_history[1] == 'h4D) && (scan_history[2][7:4] != 'hF)); // Key for P
	assign letter[16] = ((scan_history[1] == 'h15) && (scan_history[2][7:4] != 'hF)); // Key for Q
	assign letter[17] = ((scan_history[1] == 'h2D) && (scan_history[2][7:4] != 'hF)); // Key for R
	assign letter[18] = ((scan_history[1] == 'h1B) && (scan_history[2][7:4] != 'hF)); // Key for S
	assign letter[19] = ((scan_history[1] == 'h2C) && (scan_history[2][7:4] != 'hF)); // Key for T
	assign letter[20] = ((scan_history[1] == 'h3C) && (scan_history[2][7:4] != 'hF)); // Key for U
	assign letter[21] = ((scan_history[1] == 'h2A) && (scan_history[2][7:4] != 'hF)); // Key for V
	assign letter[22] = ((scan_history[1] == 'h1D) && (scan_history[2][7:4] != 'hF)); // Key for W
	assign letter[23] = ((scan_history[1] == 'h22) && (scan_history[2][7:4] != 'hF)); // Key for X
	assign letter[24] = ((scan_history[1] == 'h35) && (scan_history[2][7:4] != 'hF)); // Key for Y
	assign letter[25] = ((scan_history[1] == 'h1A) && (scan_history[2][7:4] != 'hF)); // Key for Z
	
//	reg [24:0] store;
//   reg press;
//	always @(posedge CLOCK_50)
//	begin
//		if (store != letter)
//			begin
//				press <= 1;
//				store <= letter;
//			end
//		else
//			press <= 0;
//	end
	
//	always @(posedge scan_ready)
//	begin
//		press <= 1;
//	end
//	assign ready = press;
	
	always @(*)
	begin
	    if(scan_history[2][7:4] != 'hF)
		     ready <= 1'b1;
		 else if (scan_history[2][7:4] == 'hF)
		     ready <= 1'b0;
	end
endmodule


module binary_to_alphabet(in, out);
    input [4:0] in;
	 output reg [25:0] out;
	 
	 always @(*)
	     begin
		      case(in)
				    5'b00000: out = 26'b00000000000000000000000001;
					 5'b00001: out = 26'b00000000000000000000000010;
					 5'b00010: out = 26'b00000000000000000000000100;
					 5'b00011: out = 26'b00000000000000000000001000;
					 5'b00100: out = 26'b00000000000000000000010000;
					 5'b00101: out = 26'b00000000000000000000100000;
					 5'b00110: out = 26'b00000000000000000001000000;
					 5'b00111: out = 26'b00000000000000000010000000;
					 5'b01000: out = 26'b00000000000000000100000000;
					 5'b01001: out = 26'b00000000000000001000000000;
					 5'b01010: out = 26'b00000000000000010000000000;
					 5'b01011: out = 26'b00000000000000100000000000;
					 5'b01100: out = 26'b00000000000001000000000000;
					 5'b01101: out = 26'b00000000000010000000000000;
					 5'b01110: out = 26'b00000000000100000000000000;
					 5'b01111: out = 26'b00000000001000000000000000;
					 5'b10000: out = 26'b00000000010000000000000000;
					 5'b10001: out = 26'b00000000100000000000000000;
					 5'b10010: out = 26'b00000001000000000000000000;
					 5'b10011: out = 26'b00000010000000000000000000;
					 5'b10100: out = 26'b00000100000000000000000000;
					 5'b10101: out = 26'b00001000000000000000000000;
					 5'b10110: out = 26'b00010000000000000000000000;
					 5'b10111: out = 26'b00100000000000000000000000;
					 5'b11000: out = 26'b01000000000000000000000000;
					 5'b11001: out = 26'b10000000000000000000000000;
					 default: out = 26'b00000000000000000000000000;
				endcase
		  end
endmodule

module alphabet_to_binary(in, out);
    input [25:0] in;
	 output reg [4:0] out;
	 
	 always @(*)
	     begin
		      case(in)
				    26'b00000000000000000000000001: out = 5'b00000;
					 26'b00000000000000000000000010: out = 5'b00001;
					 26'b00000000000000000000000100: out = 5'b00010;
					 26'b00000000000000000000001000: out = 5'b00011;
					 26'b00000000000000000000010000: out = 5'b00100;
					 26'b00000000000000000000100000: out = 5'b00101;
					 26'b00000000000000000001000000: out = 5'b00110;
					 26'b00000000000000000010000000: out = 5'b00111;
					 26'b00000000000000000100000000: out = 5'b01000;
					 26'b00000000000000001000000000: out = 5'b01001;
					 26'b00000000000000010000000000: out = 5'b01010;
					 26'b00000000000000100000000000: out = 5'b01011;
					 26'b00000000000001000000000000: out = 5'b01100;
					 26'b00000000000010000000000000: out = 5'b01101;
					 26'b00000000000100000000000000: out = 5'b01110;
					 26'b00000000001000000000000000: out = 5'b01111;
					 26'b00000000010000000000000000: out = 5'b10000;
					 26'b00000000100000000000000000: out = 5'b10001;
					 26'b00000001000000000000000000: out = 5'b10010;
					 26'b00000010000000000000000000: out = 5'b10011;
					 26'b00000100000000000000000000: out = 5'b10100;
					 26'b00001000000000000000000000: out = 5'b10101;
					 26'b00010000000000000000000000: out = 5'b10110;
					 26'b00100000000000000000000000: out = 5'b10111;
					 26'b01000000000000000000000000: out = 5'b11000;
					 26'b10000000000000000000000000: out = 5'b11001;
					 default: out = 5'b11111;
				endcase
		  end
endmodule

module hex_decoder(hex_digit, segments);
    input [3:0] hex_digit;
    output reg [6:0] segments;
   
    always @(*)
        case (hex_digit)
            4'h0: segments = 7'b100_0000;
            4'h1: segments = 7'b111_1001;
            4'h2: segments = 7'b010_0100;
            4'h3: segments = 7'b011_0000;
            4'h4: segments = 7'b001_1001;
            4'h5: segments = 7'b001_0010;
            4'h6: segments = 7'b000_0010;
            4'h7: segments = 7'b111_1000;
            4'h8: segments = 7'b000_0000;
            4'h9: segments = 7'b001_1000;
            4'hA: segments = 7'b000_1000;
            4'hB: segments = 7'b000_0011;
            4'hC: segments = 7'b100_0110;
            4'hD: segments = 7'b010_0001;
            4'hE: segments = 7'b000_0110;
            4'hF: segments = 7'b000_1110;   
            default: segments = 7'h7f;
        endcase
endmodule
