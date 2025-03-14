module datapath(
	input 	logic 		 clk, reset,
	input 	logic [1:0]  ResultSrc, PCSrc,
	input 	logic 		 ALUSrc,
	input 	logic 		 RegWrite,
	input 	logic [2:0]  ImmSrc,
	input 	logic [3:0]  ALUControl,
	output 	logic 		 Zero,
	output	logic			 Lt,
	output 	logic [31:0] PC,
	input 	logic [31:0] Instr,
	output 	logic [31:0] ALUResult, WriteData,
	input 	logic [31:0] ReadData);
	
	logic [31:0] PCNext, PCPlus4, PCTarget;
	logic [31:0] ImmExt;
	logic [31:0] SrcA, SrcB;
	logic [31:0] Result;
	
	// next PC logic
	// resettable flip flop, the #(32) means the input parameter
	// in this case if the number of bits we want to have
	flopr #(32) pcreg(clk, reset, PCNext, PC);
	
	adder pcadd4(PC, 32'd4, PCPlus4);
	adder pcaddbranch(PC, ImmExt, PCTarget);
	
	// mux2 #(32) pcmux(PCPlus4, PCTarget, PCSrc, PCNext);
	mux3 #(32) pcmux(PCPlus4, PCTarget, ALUResult, PCSrc, PCNext);
	
	// register file logic
	regfile rf(clk, RegWrite, Instr[19:15], Instr[24:20],
			     Instr[11:7], Result, SrcA, WriteData);
	
	extend ext(Instr[31:7], ImmSrc, ImmExt);
	
	// ALU logic
	mux2 #(32) srcbmux(WriteData, ImmExt, ALUSrc, SrcB);
	
	alu alu(SrcA, SrcB, ALUControl, ALUResult, Zero, Lt);
	
	mux3 #(32) resultmux(ALUResult, ReadData, PCPlus4, ResultSrc, Result);
	
endmodule
	