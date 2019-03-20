//`include "rotor.v"
//`include "reflector.v"

module EnigmaMachine(SW, HEX0, HEX1, KEY, CLOCK_50);
    input [9:0] SW;
	 input [3:0] KEY;
	 input CLOCK_50;
	 output [6:0] HEX0, HEX1;
	 wire [25:0] cov_out, rotor_out, ref_out, rotor_out2, cov_out2;
	 reg [4:0] to_hex;
	 
	 binary_to_alphabet b0(.in(SW[4:0]), .out(cov_out));
	 rotor r0(.in(cov_out), .out(rotor_out), .clock(CLOCK_50), .rotate(~KEY[1]), .reset(SW[9]));
	 reflector ref0(.in(rotor_out), .out(ref_out));
	 rotor r0r(.in(ref_out), .out(rotor_out2), .clock(CLOCK_50), .rotate(~KEY[1]), .reset(SW[9]));
	 alphabet_to_binary a0(.in(rotor_out2), .out(cov_out2));
	 
	 always @(posedge ~KEY[2])
	     begin
		      to_hex <= cov_out2; 	    
		  end
    
	 hex_decoder H0(
        .hex_digit(to_hex[3:0]), 
        .segments(HEX0)
        );
        
    hex_decoder H1(
        .hex_digit({3'b000, to_hex[4]}), 
        .segments(HEX1)
        );
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
					 default: out = 5'b00000;
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
