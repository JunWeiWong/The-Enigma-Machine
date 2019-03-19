module rotor(out, in, rotate);
    input [25:0] in;
    input rotate;
    output [25:0] out;
    reg [4:0] 
    localparam A = 5'b00000, B = 5'b000001, C = 5'b00010, D = 5'b00011,
               E = 5'b00100, F = 5'b00101, G = 5'b00110, H = 5'b00111,
               I = 5'b01000, J = 5'b01001, K = 5'b01010, L = 5'b01011,
               M = 5'b01100, N = 5'b01101, O = 5'b01110, P = 5'b01111,
               Q = 5'b10000, R = 5'b10001, S = 5'b10010, T = 5'b10011,
               U = 5'b10100, V = 5'b10101, W = 5'b10110, X = 5'b10111,
               Y = 5'b11000, Z = 5'b11001;
     
     reg [4:0] state;
     
     always @(posedge rotate)
     begin: state_table
          case(state)
          A: state = B;
          B: state = C;
          C: state = D;
          D: state = E;
          E: state = F;
          F: state = G;
          G: state = H;
          H: state = I;
          I: state = J;
          J: state = K;
          K: state = L;
          L: state = M;
          M: state = N;
          N: state = O;
          O: state = P;
          P: state = Q;
          Q: state = R;
          R: state = S;
          S: state = T;
          T: state = U;
          U: state = V;
          V: state = W;
          W: state = X;
          X: state = Y;
          Y: state = Z;
          Z: state = A;
          default: state = A;
          endcase
     end
    
     wire [51:0] shiftinput;
     wire [51:0] shiftoutput;
     wire [25:0] intermdiate;
     
     assign shiftinput = ({in[25:0],26'b0} >> state);
     
     default_mapping d0(
          .in(shiftinput[51:26]|shiftinput[25:0]), 
          .out(intermediate));
          
     assign shiftoutput = ({26'b0, intermediate[25:0]} << state);
     assign out = (shiftoutput[51:26]|shiftoutput[25:0]);
endmodule


module default_mapping(out, in)
     input [25:0] in;
     output [25:0] out;
     
     assign out = 26'b0;
     
     assign out[0] = in[17];
     assign out[1] = in[20];
     assign out[2] = in[12];
     assign out[3] = in[23];
     assign out[4] = in[9];
     assign out[5] = in[10];
     assign out[6] = in[5];
     assign out[7] = in[18];
     assign out[8] = in[25];
     assign out[9] = in[3];
     assign out[10] = in[11];
     assign out[11] = in[4];
     assign out[12] = in[19];
     assign out[13] = in[7];
     assign out[14] = in[21];
     assign out[15] = in[6];
     assign out[16] = in[13];
     assign out[17] = in[15];
     assign out[18] = in[24];
     assign out[19] = in[1];
     assign out[20] = in[16];
     assign out[21] = in[0];
     assign out[22] = in[8];
     assign out[23] = in[14];
     assign out[24] = in[2];
     assign out[25] = in[22];
endmodule