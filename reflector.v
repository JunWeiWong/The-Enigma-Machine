module reflector(in, out);
    input [25:0] in;
	 output [25:0] out;
	 
	 assign out[0] = in[20];
	 assign out[1] = in[22];
	 assign out[2] = in[9];
	 assign out[3] = in[14];
	 assign out[4] = in[6];
	 assign out[5] = in[2];
	 assign out[6] = in[19];
	 assign out[7] = in[10];
	 assign out[8] = in[7];
	 assign out[9] = in[4];
	 assign out[10] = in[5];
	 assign out[11] = in[16];
	 assign out[12] = in[23];
	 assign out[13] = in[12];
	 assign out[14] = in[25];
	 assign out[15] = in[0];
	 assign out[16] = in[11];
	 assign out[17] = in[1];
	 assign out[18] = in[21];
	 assign out[19] = in[15];
	 assign out[20] = in[13];
	 assign out[21] = in[24];
	 assign out[22] = in[18];
	 assign out[23] = in[17];
	 assign out[24] = in[8];
	 assign out[25] = in[3];

endmodule