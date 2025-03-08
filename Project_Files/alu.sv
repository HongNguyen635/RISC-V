module alu (
	input 	logic [31:0] a,
	input 	logic [31:0] b,
	input 	logic [3:0]  alucontrol,
	output 	logic [31:0] result,
	output	logic			 zero);
	
	// condinvb = check to invert b if subtract for 2s compliment
	logic [31:0] condinvb, sum;
	logic signed [31:0] shamt, shift;
	logic cout; // carry out of adder
	logic overflow; // check for overflow
	
	assign condinvb = alucontrol[0] ? ~b : b;
	
	assign {cout, sum} = a + condinvb + alucontrol[0];
	
	// see p.251 for overflow diagram
	assign overflow = ~(alucontrol[0] ^ b[31] ^ a[31]) & 
							(a[31] ^ sum[31]) & ~alucontrol[1];
							
	// zero flag
	assign zero = (sum == 32'b0);
	
	// values for shifting
	assign shamt = {{27{1'b0}}, {b[4:0]}};
	assign shift = a;
	
	
	always_comb
		case (alucontrol)
			4'b0000: result = sum; // add
			4'b0001: result = sum; // subtract
			4'b0010: result = a & b; // and
			4'b0011: result = a | b; // or
			4'b0101: result = (sum[31] ^ overflow); // slt
			4'b0110: result = shift << shamt; // slli
			4'b1110: result = shift >>> shamt; // srai
			4'b1000: result = shift >> shamt; // srli
			default: result = 32'bx; // undefined
		endcase
		
endmodule