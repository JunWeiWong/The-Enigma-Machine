module rotor(out, in, clock, rotate, set, set_state, state, num);
    input [25:0] in;
    input rotate, clock, set;
	 input [4:0] set_state;
    output [25:0] out;
    localparam A = 5'b00000, B = 5'b000001, C = 5'b00010, D = 5'b00011,
               E = 5'b00100, F = 5'b00101, G = 5'b00110, H = 5'b00111,
               I = 5'b01000, J = 5'b01001, K = 5'b01010, L = 5'b01011,
               M = 5'b01100, N = 5'b01101, O = 5'b01110, P = 5'b01111,
               Q = 5'b10000, R = 5'b10001, S = 5'b10010, T = 5'b10011,
               U = 5'b10100, V = 5'b10101, W = 5'b10110, X = 5'b10111,
               Y = 5'b11000, Z = 5'b11001;
     
     output reg [4:0] state;
	  reg [4:0] next_state;
//	  initial state = set;
//	  always @(*)
//	  begin
//	      if (set)
//			    state <= set_state;
//	  end
	  
     always @(posedge rotate)
     begin
	       case(state)
				 A: next_state = B;
				 B: next_state = C;
				 C: next_state = D;
				 D: next_state = E;
				 E: next_state = F;
				 F: next_state = G;
				 G: next_state = H;
				 H: next_state = I;
				 I: next_state = J;
				 J: next_state = K;
				 K: next_state = L;
				 L: next_state = M;
				 M: next_state = N;
				 N: next_state = O;
				 O: next_state = P;
				 P: next_state = Q;
				 Q: next_state = R;
				 R: next_state = S;
				 S: next_state = T;
				 T: next_state = U;
				 U: next_state = V;
				 V: next_state = W;
				 W: next_state = X;
				 X: next_state = Y;
				 Y: next_state = Z;
				 Z: next_state = A;
				 default: next_state = A;
			endcase
	  end
  
     output reg num;
	  always @(*)
     begin
			if (set == 1'b1)
			    begin
				    state <= set_state;
					 num <= 0;
			    end
			else if ((set == 1'b0) && (rotate == 1))
				 begin
			        if (num == 0)
						state <= next_state - 1'b1;
					  else 
						state <= next_state;
					num <= 1;
				 end
	  end
	  
     wire [51:0] shiftinput;
     wire [51:0] shiftoutput;
     wire [25:0] intermediate;
     
     assign shiftinput = ({in[25:0],26'b0} >> state);
     
     default_mapping d0(
          .in(shiftinput[51:26]|shiftinput[25:0]), 
          .out(intermediate));
          
     assign shiftoutput = ({26'b0, intermediate[25:0]} << state);
     assign out = (shiftoutput[51:26]|shiftoutput[25:0]);
endmodule


module default_mapping(in, out);
     input [25:0] in;
     output [25:0] out;
     
     //assign out = 26'b0;
     
     assign out[0] = in[17];
     assign out[1] = in[20];
     assign out[2] = in[12];
     assign out[3] = in[23];
     assign out[4] = in[9];
     assign out[5] = in[10];
     assign out[6] = in[15];
     assign out[7] = in[18];
     assign out[8] = in[25];
     assign out[9] = in[4];
     assign out[10] = in[5];
     assign out[11] = in[24];
     assign out[12] = in[2];
     assign out[13] = in[16];
     assign out[14] = in[21];
     assign out[15] = in[6];
     assign out[16] = in[13];
     assign out[17] = in[0];
     assign out[18] = in[7];
     assign out[19] = in[22];
     assign out[20] = in[1];
     assign out[21] = in[14];
     assign out[22] = in[19];
     assign out[23] = in[3];
     assign out[24] = in[11];
     assign out[25] = in[8];
endmodule