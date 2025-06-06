module alu (
	input 	logic [31:0] a,
	input 	logic [31:0] b,
	input 	logic [3:0]  alucontrol,
	output 	logic [31:0] result,
	output	logic			 zero,
	output	logic			 lt
	);
	
	// condinvb = check to invert b if subtract for 2s compliment
	logic [31:0] condinvb, sum;
	logic signed [31:0] shamt, shamtr, shift;
	logic cout; // carry out of adder
	logic overflow; // check for overflow
	
	assign condinvb = alucontrol[0] ? ~b : b;
	
	assign {cout, sum} = a + condinvb + alucontrol[0];
	
	// see p.251 for overflow diagram
	assign overflow = ~(alucontrol[0] ^ b[31] ^ a[31]) & 
							(a[31] ^ sum[31]) & ~alucontrol[1];
							
	// zero flag
	assign zero = (sum == 32'b0);

	//less than flag
	assign lt = (sum[31] ^ overflow);
	
	// values for shifting
	assign shamt = {{27{1'b0}}, {b[4:0]}}; // extend b to ignore funct7b5
	assign shamtr = b; // shift ammount for register shift, no extension
	assign shift = a; // shift target, signed for arithmetic shift, arithmetic shift does not work otherwise
	
	
	always_comb
		case (alucontrol)
			4'b0000: result = sum; // add
			4'b0001: result = sum; // subtract
			4'b0010: result = a & b; // and
			4'b0011: result = a | b; // or
			4'b0100: result = a ^ b; // xor
			4'b0101: result = (sum[31] ^ overflow); // slt
			4'b0110: result = shift << shamt; // slli
			4'b0111: result = shift >> shamt; // srli
			4'b1000: result = shift >>> shamt; // srai
			4'b1001: result = shift << shamtr; // sll
			4'b1010: result = shift >> shamtr; // srl
			4'b1011: result = shift >>> shamtr; // sra
			default: result = 32'bx; // undefined
		endcase
		
endmodule