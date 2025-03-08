module controller (
	input		logic [6:0] op,
	input		logic [2:0] funct3,
	input		logic			funct7b5,
	input		logic			Zero,
	output	logic [1:0] ResultSrc,
	output 	logic			MemWrite,
	output 	logic	[1:0]	PCSrc, 
	output	logic			ALUSrc, RegWrite, Jump,
	output	logic	[1:0] ImmSrc,
	output	logic [3:0] ALUControl);
	

	logic [1:0] ALUOp;
	logic			Branch, Jalr;
	
	maindec md (op, ResultSrc, MemWrite, Branch, ALUSrc, RegWrite, Jump, ImmSrc, ALUOp, Jalr);
	
	aludec ad (op[5], funct3, funct7b5, ALUOp, ALUControl);
	
	// assign PCSrc = Branch & Zero | Jump;
	
	always_comb begin
		if (Jalr)
			PCSrc = 2'b10;
		else if (Branch & Zero | Jump)
			PCSrc = 2'b01;
		else
			PCSrc = 2'b00;
	end
	
endmodule