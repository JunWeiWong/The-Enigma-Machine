module EnigmaMachine(SW, LEDR, CLOCK_50);
    input [9:0] SW;
	 output [9:0] LEDR;
	 
	 rotor r0();
	 reflector ref0();
	 rotor r0r();
endmodule