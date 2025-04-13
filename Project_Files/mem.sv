// instruction memory
module imem (
	input 	logic [31:0] address,
	output 	logic [31:0] rd);
	
	// 64 entries, each 32 bits
	logic [31:0] RAM[63:0];
	
	// initial has 1 statement can omit the begin and end
	initial
		// read file into RAM, the file content is in hex
		// to read binary, use readmemb
		// $readmemh("D:/Capstone/RiscV_Current/riscvtest.txt", RAM);
		$readmemh("D:/Capstone/RiscV_Current/instruction.bin", RAM);
		
	assign rd = RAM[address[31:2]]; // word aligned
	
endmodule

// data memory
module dmem (
	input 	logic 		 clk, we,
	input 	logic [31:0] a, wd,
	output 	logic [31:0] rd);
	
	// 64 entries, each 32 bits
	logic [31:0] RAM[63:0];
	
	assign rd = RAM[a[31:2]]; // word aligned
	
	// ff = flip flop, sequential logic
	always_ff @(posedge clk)
		// if write enable
		if (we) RAM[a[31:2]] <= wd;
		
endmodule

// instruction memory - reading from external EEPROM
module imemEEPROM (
	input 	logic [31:0] address,
	output 	logic [31:0] rd);
	
	// TODO:
	// similar to the internal imem module above
	// modifying it in a way that it reads 4 consecutive bytes
	// then assemble into a 32-bit word
	// then assign it to rd (just like the last line
	// in the internal imem module).
	//
	// Will need a state machine that is very
	// similar to the one in EEPROM read code
	// in the refactor SV file folder.
	
endmodule

// data memory
module dmemSRAM (
	input 	logic 		 clk, we,
	input 	logic [31:0] a, wd,
	output 	logic [31:0] rd);
	
	// TODO:
	// similar to the internal dmem module
	// here dmem is the SRAM
	// for read, do the similar thing as
	// EEPROM. Read byte wise, then assembly it
	// into a 32-bit word.
	// for write, divide the 32-bit input word
	// and write each byte to the SRAM
	//
	// will need a state machine that is very similar
	// to imem_eeprom above.
		
endmodule

// register file
module regfile (
	input 	logic 		clk,
	input 	logic 		we3,
	input 	logic [4:0] a1, a2, a3,
	input 	logic [31:0] wd3,
	output 	logic [31:0] rd1, rd2);
	
	// 32 registers each 32 bits
	logic [31:0] rf[31:0];
	
	// three ported register file
	// read two ports combinationally (A1/RD1, A2/RD2)
	// write third port on rising edge of clock (A3/WD3/WE3)
	// register 0 hardwired to 0
	always_ff @(posedge clk)
		if (we3) rf[a3] <= wd3;
	
	assign rd1 = (a1 != 0) ? rf[a1] : 0;
	assign rd2 = (a2 != 0) ? rf[a2] : 0;
	
endmodule