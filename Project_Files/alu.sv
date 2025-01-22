module alu (
	input 	logic [31:0] a,
	input 	logic [31:0] b,
	input 	logic [2:0]  alucontrol,
	output 	logic [31:0] result,
	output	logic			 zero);
	
	// condinvb = check to invert b if subtract for 2s compliment
	logic [31:0] condinvb, sum;
	logic cout; // carry out of adder
	logic overflow; // check for overflow
	
	assign condinvb = alucontrol[0] ? ~b : b;
	
	assign {cout, sum} = a + condinvb + alucontrol[0];
	
	// see p.251 for overflow diagram
	assign overflow = ~(alucontrol[0] ^ b[31] ^ a[31]) & 
							(a[31] ^ sum[31]) & ~alucontrol[1];
							
	// zero flag
	assign zero = (sum == 32'b0);
	
	always_comb
		case (alucontrol)
			3'b000: result = sum; // add
			3'b001: result = sum; // subtract
			3'b010: result = a & b; // and
			3'b011: result = a | b; // or
			3'b101: result = (sum[31] ^ overflow); // slt
			default: result = 32'bx; // undefined
		endcase
		
endmodule