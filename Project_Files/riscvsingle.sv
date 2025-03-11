module riscvsingle (
	input 	logic clk, reset,
	output 	logic [31:0] PC,
	input 	logic [31:0] Instr,
	output 	logic MemWrite,
	output 	logic [31:0] ALUResult, WriteData,
	input 	logic [31:0] ReadData);
	
	logic 		ALUSrc, RegWrite, Jump, Zero, Lt;
	logic [1:0] ResultSrc;
	logic [2:0] ImmSrc;
	logic [3:0] ALUControl;
	logic	[1:0]	PCSrc; // the textbook is missing this signal
	
	controller c(Instr[6:0], Instr[14:12], Instr[30], Zero, Lt, 
					 ResultSrc, MemWrite, PCSrc, ALUSrc, RegWrite, Jump,
					 ImmSrc, ALUControl);
					 
	datapath dp(clk, reset, ResultSrc, PCSrc, ALUSrc, RegWrite,
					ImmSrc, ALUControl, Zero, Lt, PC, Instr,
					ALUResult, WriteData, ReadData);
endmodule