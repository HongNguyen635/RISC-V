// Name:
// 
// Description:
//
// [fill in later once finsihed]

module riscVfpga (
	input                      ADC_CLK_10,	MAX10_CLK1_50,	MAX10_CLK2_50,
	output		     [7:0]		HEX0,	HEX1,	HEX2,	HEX3,	HEX4,	HEX5,
	input 		     [1:0]		KEY,
	output		     [9:0]		LEDR,
	input 		     [9:0]		SW,
	inout 		    [35:0]		RISCV_GPIO	);
	
	// TODO: re-oraganize this logic code later
	// everything is the same as the topLevel.sv file
	// basically topLevel.sv is the file for simulation.
	// this file is for loading onto the fpga.
	logic 		 clk, reset;
	logic [31:0] WriteData, DataAdr;
	logic 		 MemWrite;
	
	logic [31:0] PC, Instr, ReadData;
	
	// instantiate processor and memories
	riscvsingle rvsingle(clk, reset, PC, Instr, MemWrite, DataAdr, WriteData, ReadData);
	
	// instruction memory & data memory
	imemEEPROM imem(PC, Instr);
	dmemSRAM dmem(clk, MemWrite, DataAdr, WriteData, ReadData);
	
	
	
	// TODO: this section is for mapping the correct GPIO pin to
	// the logic analyzer.
	// Need to:
	// - Map the GPIO number from RISCV_GPIO to the correct signal (AS, RD, WE, etc.)
	// - Sounds easy but the hard part is figure out which is which, because the current
	// schematic doesn't match with the pcb layout.
	//
	// To keep consistency, use the logic signals define from line 8 -> 23.
	// if you want to add anything extra, ask in the discord 1st so that
	// everyone can keep track of it.
	
	// [code goes here]
	
	
	
	// TODO: this section is for mapping the address & data on the hex display
	// Currently, although our address and data is 32 bit, we only have enough
	// pins for 16 bit for the LA.
	// For address, I don't think we'd ever exceed the 2^16 address range.
	// Current thinking:
	// - Maybe have 1 switch for switching between data & address,
	// show 16 bit at a time.
	// - Another switch: off = 1st half of the 16 bit, on = last half
	// - Since 16 bit only uses 4 hex displays, the remaining 2 would be
	// either Ad (stands for Address) and Da (stands for Data) to indicate
	// the operation.
	//
	// To keep consistency, use the logic signals define from line 8 -> 23.
	// if you want to add anything extra, ask in the discord 1st so that
	// everyone can keep track of it.
	
	// [code goes here]
	
	
	
	
	// TODO: put everything together.
	// Once imem_eeprom, dmem_sram, LA, FPGA sections are finsihed,
	// this should be simple as it only maps which clock we want to use
	// (manual, automatic).
	// So this will be the last thing we do.


endmodule : riscVfpga

module seg7(input [3:0] hex, output [7:0] segment);
    reg [7:0] leds;
    always@(*) begin
        case(hex)
            0: leds =  8'b00111111;	// 0 image
            1: leds =  8'b00000110;	// 1 image
            2: leds =  8'b01011011;	// 2 image
            3: leds =  8'b01001111;	// 3 image
            4: leds =  8'b01100110;	// 4 image
            5: leds =  8'b01101101;	// 5 image
            6: leds =  8'b01111101;	// 6 image
            7: leds =  8'b00000111;	// 7 image
            8: leds =  8'b01111111;	// 8 image
            9: leds =  8'b01101111;	// 9 image
            10: leds = 8'b01110111;	// A image
            11: leds = 8'b01111100;	// b image
            12: leds = 8'b00111001;	// C image
            13: leds = 8'b01011110;	// d image
            14: leds = 8'b01111001;	// E image
            15: leds = 8'b01110001;	// F image
        endcase
        end
	assign segment = ~leds;		// invert and copy to outputs
endmodule : seg7