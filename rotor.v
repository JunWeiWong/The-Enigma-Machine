module rotor(out, in, clock, rotate, reset);
    input [25:0] in;
    input rotate, clock, reset;
    output [25:0] out;
    localparam A = 5'b00000, B = 5'b000001, C = 5'b00010, D = 5'b00011,
               E = 5'b00100, F = 5'b00101, G = 5'b00110, H = 5'b00111,
               I = 5'b01000, J = 5'b01001, K = 5'b01010, L = 5'b01011,
               M = 5'b01100, N = 5'b01101, O = 5'b01110, P = 5'b01111,
               Q = 5'b10000, R = 5'b10001, S = 5'b10010, T = 5'b10011,
               U = 5'b10100, V = 5'b10101, W = 5'b10110, X = 5'b10111,
               Y = 5'b11000, Z = 5'b11001;
     
     reg [4:0] state;
	  
	  initial begin
	       state <= A;
	  end
	  
     always @(posedge rotate)
     begin
	       case(state)
				 A: state = reset ? A : B;
				 B: state = reset ? A : C;
				 C: state = reset ? A : D;
				 D: state = reset ? A : E;
				 E: state = reset ? A : F;
				 F: state = reset ? A : G;
				 G: state = reset ? A : H;
				 H: state = reset ? A : I;
				 I: state = reset ? A : J;
				 J: state = reset ? A : K;
				 K: state = reset ? A : L;
				 L: state = reset ? A : M;
				 M: state = reset ? A : N;
				 N: state = reset ? A : O;
				 O: state = reset ? A : P;
				 P: state = reset ? A : Q;
				 Q: state = reset ? A : R;
				 R: state = reset ? A : S;
				 S: state = reset ? A : T;
				 T: state = reset ? A : U;
				 U: state = reset ? A : V;
				 V: state = reset ? A : W;
				 W: state = reset ? A : X;
				 X: state = reset ? A : Y;
				 Y: state = reset ? A : Z;
				 Z: state = reset ? A : A;
				 default: state = A;
				 endcase
     end
    
//	  always @(posedge clock)
//     begin
//			if (reset == 1'b1)
//				s <= A;
//			else 
//				s <= state;
//		end
	  
	  
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