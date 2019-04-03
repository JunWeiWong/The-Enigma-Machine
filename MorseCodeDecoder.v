module MorseCodeDecoder(clock, in, load, show, reset, out);
    input clock;
	 input[4:0] in;
	 input show, load, reset;
	 output out;
	 wire pulse;
	 wire[30:0] cycle;
	 wire[30:0] wire0;
	 wire[13:0] wire1;
	 wire[139:0] wire2;
	 assign cycle = 25000000;
	 
	 RateDivider r0(.speed(cycle), .clock(clock), .reset_n(reset), .count(wire0));
	 assign pulse = (wire0 == 0) ? 1 : 0;
	 LUT l0(.in(in), .out(wire1));
	 buffer b0(.in(wire1), .load(load), .clock(clock), .message(wire2));
	 Shifter s0(.clock(clock), .enable(pulse), .show(show), .reset(reset), .d(wire2), .q(out));
endmodule

module LUT(in, out);
    input[4:0] in;
    output reg [13:0] out;

    always @(*)
    begin
        case(in)
            5'b00000: out = 14'b10111000000000; //A
            5'b00001: out = 14'b11101010100000; //B
            5'b00010: out = 14'b11101011101000; //C
            5'b00011: out = 14'b11101010000000; //D
            5'b00100: out = 14'b10000000000000; //E
            5'b00101: out = 14'b10101110100000; //F
            5'b00110: out = 14'b11101110100000; //G
            5'b00111: out = 14'b10101010000000; //H
            5'b01000: out = 14'b10100000000000; //I
			   5'b01001: out = 14'b10111011101110; //J
			   5'b01010: out = 14'b11101011100000; //K
			   5'b01011: out = 14'b10111010100000; //L
			   5'b01100: out = 14'b11101110000000; //M
			   5'b01101: out = 14'b11101000000000; //N
			   5'b01110: out = 14'b11101110111000; //O
			   5'b01111: out = 14'b10111011101000; //P
			   5'b10000: out = 14'b11101110101110; //Q
			   5'b10001: out = 14'b10111010000000; //R
			   5'b10010: out = 14'b10101000000000; //S
			   5'b10011: out = 14'b11100000000000; //T
			   5'b10100: out = 14'b10101110000000; //U
			   5'b10101: out = 14'b10101011100000; //V
			   5'b10110: out = 14'b10111011100000; //W
			   5'b10111: out = 14'b11101010111000; //X
			   5'b11000: out = 14'b11101011101110; //Y
			   5'b11001: out = 14'b11101110101000; //Z
			   default: out = 14'b00000000000000;
        endcase
    end
    
endmodule

module RateDivider(speed, clock, reset_n, count);
    input[30:0] speed;
	 input clock, reset_n;
    output reg[30:0] count;
	 
	 always @(posedge clock)
    begin
        if (reset_n == 1'b0)
            count <= speed - 1'b1;
        else
            begin
                if (count == 0)
                    count <= speed - 1'b1;
                else
                    count <= count - 1'b1;
				end
    end
endmodule

module Shifter(clock, enable, show, reset, d, q);
    input clock, enable, show, reset;
    input [139:0] d;
    output q;
    reg [139:0] code;

    always @(posedge clock)
    begin
        if (reset == 1'b0)
            code = 0;
        else if (show == 1'b1)
            code = d;
        else if (enable == 1'b1)
            code = code << 1;
    end

    assign q = code[139];

endmodule

module buffer(in, load, clock, message);
    input[13:0] in;
	 input load, clock;
	 output reg [139:0] message;
	 
	 always @(posedge clock)
	 begin
	     if (load)
		      message[13:0] = in;
				message = message >> 14;
	 end

endmodule