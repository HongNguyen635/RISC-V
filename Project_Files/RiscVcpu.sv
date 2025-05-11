//=======================================================
//  Overview:
//
//  Authors: Hong Nguyen, Donald Philips, Cheng-Wei Lin (Eric), Andrew Hanson
//
//  What this file does:
//  Use SW9 to control internal/external memory:
//  - SW9 on (up): external memory
//  - SW9 off (down): internal memory
//
//  For word clock:
//  - Automatically clock slows down to 200 kHz
//  - Word clock for internal memory runs at 200 kHz / 4 = 50 kHz
//  - Word clock for external memory runs at 200 kHz / 60 = 3.33 kHz
//  - Slow clock is for debugging purposes and accomodating slow
//  external memory. Increase the clock speed if needed at line []
//
//  When sampling with the logic analyzer, the sampling rate needs
//  to be at least ~2 - 3x higher than the clock for it to work.
//
//=======================================================

module RiscVcpu(

	//////////// CLOCK //////////
	input 		          		ADC_CLK_10,
	input 		          		MAX10_CLK1_50,
	input 		          		MAX10_CLK2_50,

	//////////// SEG7 //////////
	output		     [7:0]		HEX0,
	output		     [7:0]		HEX1,
	output		     [7:0]		HEX2,
	output		     [7:0]		HEX3,
	output		     [7:0]		HEX4,
	output		     [7:0]		HEX5,

	//////////// KEY //////////
	input 		     [1:0]		KEY,

	//////////// LED //////////
	output		     [9:0]		LEDR,

	//////////// SW //////////
	input 		     [9:0]		SW,

	//////////// GPIO, GPIO connect to GPIO Default //////////
	inout 		    [35:0]		GPIO
);


//=======================================================
//  Structural coding
//=======================================================

	// logic for use with the risc-v processor
	logic 		 clk, reset, externalReset, isExternalMem, isXmemRead;
	logic [31:0] WriteData, DataAdr;
	logic 		 MemWrite;
	
	logic [31:0] PC, Instr, ReadData;
	wire 	[30:0] xGPIO;
	logic	[7:0] xDataIn, xDataOut;
	
	// instantiate processor and memories
	riscvsingle rvsingle(clk, reset, PC, Instr, MemWrite, DataAdr, WriteData, ReadData);
	

	// instruction memory & data memory
	// imemEEPROM imem(PC, Instr);
	// dmemSRAM dmem(clk, MemWrite, DataAdr, WriteData, ReadData);
   imem imem(PC, iInstr);
	dmem dmem(clk, MemWrite, DataAdr, WriteData, iReadData);

   // external memory
   xmem xmem(autoClock, reset, isExternalMem, MemWrite, PC, xInstr, DataAdr, WriteData, xReadData, xClock, xGPIO, isXmemRead, xDataIn, xDataOut);

   // For all variables common to imem and dmem, variable below use to select
   // the correct ones. Anything with prefix i- is internal mem. Anything with
   // prefix x- is external mem.
   logic iClock, xClock, autoClock;
	logic [31:0] iInstr, xInstr, iReadData, xReadData;
	
	// debounced manual clock
	wire debouncedClk, NCLK, ClockReset;

   // 32-bit clock divider
   reg [31:0] ClockDivider;

   // this portion is debouncing the manual clock
   always @(*) begin
		if(debouncedClk == 0) 
            ClockDivider = 0;
       else 
            ClockDivider = ClockDivider + 1;	
   end

   // note key 1 is manual clock
   assign ClockReset = ClockDivider[28];	// chose delay time
   assign debouncedClk = ~(KEY[1] & NCLK);	// half of RS NAND FF
   assign NCLK = ~(ClockReset & debouncedClk);		// other half of RS NAND


	// this portion generate a 200 kHz Hz clock (the auto clock)
	// =====
	// How it works:
	// FPGA Clock 1 is 50 MHz = 20 ns period
	// this counts down from 250 every period,
	// meaning it divides the clock. So time taken to reach 0
	// = 250 * 20 ns = 5 us
	// the auto clock toggle every 5 us -> period = 10 us
	// -> f = 1/10 us = 200 kHz
	logic [25:0] SCLK;
	initial SCLK = 250; // 200 kHz clock

	// trigger on the pos edge of the DE10 50 MHz clock
	always @(posedge MAX10_CLK1_50) begin
	  // count down
	  SCLK = SCLK-1;

	  // if it reaches 0
	  if(SCLK==0) begin
			// reset the counter
			SCLK = 250;

			// and toggle the auto clock
			autoClock <= ~autoClock; 
		end
	end	

    // clock selection based on switch 9
    // if switch 9 is on = use the word clock by xMem
    // else use the iClock
    always @(*) clk <= (isExternalMem ? xClock : iClock);
    
	 // Debug: uncoment this line for manual clock
	 // always @(*) clk <= (SW[8] ? iClock : debouncedClk);
	 // always @(*) clk <= (SW[8] ? iClock : xClock);
	
	
	// ==========================
   assign Instr = isExternalMem ? xInstr : iInstr;
   
	assign ReadData = isExternalMem ? xReadData : iReadData;

   // using SW9 to select between internal and external memory
   assign isExternalMem = SW[9];

   // reset: either using key 1 or the external button on the PCB
   // assign reset = ~KEY[0] | ~externalReset;
	assign reset = ~KEY[0];
	
	// Clock
	assign GPIO[35] = clk;
	
   // External PCB reset button maps to GPIO
	// assign externalReset = GPIO[31];   
	assign GPIO[31] = ~KEY[0];
    
   // address bus
   assign GPIO[23:8] = isExternalMem ? xGPIO[23:8] : PC[15:0];

   // data bus
   // the "data" here is basically the 32 bit instruction, not the data load/store to memory
   // The data bus is 1 byte, so you need 4 clock cycles to have 4 bytes
   // The "word" clock is the one that we will output to the LA
   // Here, the word clock is denoted as "iClock"
	logic [2:0] currentClockCount;
   logic [7:0] iDataBus;
	logic RD;

	// always start with high edge
	// Instruction becomes unstable after the clock is high
	// because it needs to fetch the instruction from memory
	// so we'll only output after when the clock is low
	// (give enough time to fetch)
	initial iClock = 1'b1;
	always @(posedge autoClock) begin
	  if (!isExternalMem) begin
			case (currentClockCount)
					  0: begin
							// enable Read (valid data) only when clock is high
							RD <= iClock ? 1'b1 : 1'b0;
							iDataBus <= iClock ? Instr[7:0]: 8'b00000000;
					  end
					  1: begin
							iDataBus <= iClock ? Instr[15:8] : 8'b00000000;
					  end
					  2: begin
							iDataBus <= iClock ? Instr[23:16] : 8'b00000000;
					  end
					  3: begin
							iDataBus <= iClock ? Instr[31:24] : 8'b00000000;
					  end
					  4: begin
							RD <= 1'b0; // disable read (invalid)
							iClock <= ~iClock;
					  end
			endcase

			if (currentClockCount == 4)
					  currentClockCount <= 0;
			else
					  currentClockCount <= currentClockCount + 1'b1;
		end
	end
	
	assign GPIO[7:0] = isExternalMem ? (isXmemRead ? 8'bz : xDataOut) : iDataBus;
	assign xDataIn = GPIO[7:0];
	
	// GPIO 27 is the RD line.
	assign GPIO[27] = isExternalMem ? xGPIO[27] : ~RD;

   // for other control signals, only when using external mem
   // that these signals are valid
   // else, when use internal mem, it'll stay in tristate
   assign GPIO[30:28] = isExternalMem ? xGPIO[30:28] : 7'bZ;
	assign GPIO[26:24] = isExternalMem ? xGPIO[26:24] : 7'bZ;
	
	
	// ====== DEBUG SECTION ======
    // Use hex displays to show debug info
	// Use SW[1:0] to control which data to show
	// 0: Addr bus [15:0]
	// 1: Instr[15:0]
	// 2: Instr[31:16]
	logic [3:0] hex3In, hex2In, hex1In, hex0In;
	seg7 s3(hex3In, HEX3);
	seg7 s2(hex2In, HEX2);
	seg7 s1(hex1In, HEX1);
	seg7 s0(hex0In, HEX0);

	always @(*) begin
		case (SW[1:0])
			2'b00: begin
				// hex3In = PC[15:12];
				// hex2In = PC[11:8];
				// hex1In = PC[7:4];
				// hex0In = PC[3:0];
				hex3In = 4'h0;
				hex2In = 4'h0;
				hex1In = GPIO[23:16];
				hex0In = GPIO[15:8];
			end
			2'b01: begin
				// hex3In = Instr[15:12];
				// hex2In = Instr[11:8];
				// hex1In = Instr[7:4];
				// hex0In = Instr[3:0];
				hex3In = 4'h0;
				hex2In = GPIO[29:26];
				hex1In = GPIO[7:4];
				hex0In = GPIO[3:0];
			end
			2'b10: begin
				hex3In = Instr[31:28];
				hex2In = Instr[27:24];
				hex1In = Instr[23:20];
				hex0In = Instr[19:16];
			end
			default: begin
				hex3In = 4'hF;
				hex2In = 4'hF;
				hex1In = 4'hF;
				hex0In = 4'hF;
			end
		endcase
	end

	assign LEDR = PC[9:0];
	

endmodule : RiscVcpu

// read from external memory
module xmem(input 	logic defaultMAXclock, reset, xMemEnable, isSramWrite,
				input		logic	[31:0] PC, 
				output	logic [31:0] Instr,
				input 	logic [31:0] DataAdr, WriteData, 
				output 	logic [31:0] ReadData,
				output	logic		 	 wordClk,
				inout   	wire	[30:0] GPIO,
				output	logic			 isXmemRead,
				input 	logic [7:0]	 dataIn,
				output   logic [7:0]  dataOut);

	// Explanation of the ports:
	// defaultMAXclock: the default 50 MHz clock from the FPGA or any auto clock
	// reset : reset signal
	// xMemEnable: enable the use of external memory (aka EEPROM and SRAM)
	// isSramWrite: if we should write to sram (aka dmem)
	// PC: program counter, next PC = PC + 4
	// Instr: 32 bits instruction after assembly the byte from EEPROM
	// DataAdr: the address of sram to use for lw/sw
	// Write Data: the data to write to sram
	// Read Data: the data read from SRAM
	// wordClck: the actual clock that will be output to the LA. Flip every 60 states (20 ns * 60 = 1.2 us = ~833 KHz)
	// GPIO: pins mapping from DE10 to LA
	// isXmemRead: is xmem doing external read operation (data comes in)
	// dataIn: data comes from the xMem
	// dataOut: data comes out of the xMem module

	// =====================

	//  PROGRAM LOGIC:
	//
	// Program counter = word address
	// Every time you increment to the next instruction, next PC = PC + 4
	// To find the byte address:
	// imem_addr = PC[31:2] * 4 + currentByte
	//
	// To get this working, we need to 1st read from the EEPROM
	// 4 bytes & assemble it to retrieve the instruction
	// 
	// EEPROM read, safe time is 200 ns.
	// 1st select the ROM (ROM CS)
	// Enable the RD output (active low)
	//
	// Then, we detect if the operation is a load or store word from
	// the retrieved instruction.
	// A load has opcode: 7'b0000011
	// A store has opcode: 7'b0100011
	// 
	// Because of the nature of the EEPROM flash program
	// We use little endian = {byte 3 + byte 2 + byte 1 + byte 0}
	// Because say if we have an instruction: 0x00500113
	// It'll be load into memory as follows:
	// Byte 0: 13
	// Byte 1: 01
	// Byte 2: 50
	// Byte 3: 00
	//
	// In RISC-V register format, opcode is the [6:0] of the instruction.
	// So the opcode info stays in the 4th byte, retrieves as byte4[6:0]
	//
	// note that load and store is a word operation
	// If it is a load operation, read from SRAM
	// If it is a store operation, write to SRAM
	// For SRAM, the input Data Address is byte address
	//
	// SRAM read/write safe time is 200 ns
	// 1st enable the RAM CS
	// for write, enable MemWrite -> enable AS and DS -> wait 200 ns
	// for read, disable MemWrite -> enable AS and DS -> wait 200 ns
	//
	// At the end, we flip the word clock.
	// Since the clock needs to have 50% duty cycle,
	// we use the total time to read from eeprom & do either a read/write
	// to sram as 1 cycle (meaning flip the clock whenever we go pass
	// half the total states in the state machine).
	//
	// =====================

	// Variables declaration
	logic WR, RD, ROM, RAM, AS, DS;
	logic [1:0] currentByteCount;

	// Byte0 ~ 3 is used to temporarily store the 4 bytes read
	logic [7:0] eepromByte0, eepromByte1, eepromByte2, eepromByte3;
	logic [7:0] sramByte0, sramByte1, sramByte2, sramByte3;

	// Calculate the address needed first
	logic[31:0] memAddress;

	// basically detects if it's the eeprom or the sram turns
	// to use the address bus
	// Note that PC is word aligned (increment by 4 every cycle)
	assign memAddress = isSramOp ? {DataAdr[31:2], currentByteCount} : {PC[31:2], currentByteCount};
	// assign memAddress = 32'h00000000;
	
	// store the data bus
	logic[31:0] dataBus;
	
	// GPIO for control signals
	assign GPIO[30] = 1'b1;	  // write enable (WE), active low, doesn't write to ROM in this case
	assign GPIO[29] = ~ROM;   // rom select, active low
	assign GPIO[28] = ~RAM;   // ram select, active low
	assign GPIO[27] = ~RD;    // read, active low
	assign GPIO[26] = ~WR;    // write to RAM, active low
	assign GPIO[25] = ~AS;    // address strobe, active low
	assign GPIO[24] = 1'b0;   // A16 - not used, keep high for now
	assign GPIO[23:16] = xMemEnable ? memAddress[15:8] : 8'bZ;	// 2nd lower byte
	assign GPIO[15:8]  = xMemEnable ? memAddress[7:0] : 8'bZ;	// lower byte
	
	// write only when data strobe and memWrite is valid
	// this is an internal signal. It is NOT the same as isSramWrite.
	// MemWrite here is use to assert the read/write to eeprom and sram
	logic MemWrite;
	
	// if write to memory, use the databus, else tristate
	// (for data from memory to come in)
	assign GPIO[7:0] = (MemWrite ? dataBus : 8'bZ);
	assign dataOut = dataBus;
	
	assign WR = (DS & MemWrite);
	
	// read only when data strobe valid but not currently write to mem
	assign RD = (DS & ~MemWrite);
	
	// signal to external when xmem is doing a read op
assign isXmemRead = ~MemWrite;


	// state variables
	// 7 bits = 128 possible states. Only use 120 states.
	logic [6:0] currentState;

	// Lower 7 bits of instruction is the opcode
	// A load has opcode: 7'b0000011
	// A store has opcode: 7'b0100011
	logic isLoad, isStore, isSramOp;
	assign isLoad = (eepromByte0[6:0] == 7'b0000011);
	assign isStore = (eepromByte0[6:0] == 7'b0100011);
	
	// do sram op whenever we have load or store
	assign isSramOp = isLoad | isStore;

	// =====================

	// starts with clock = 1;
	initial wordClk = 1;
	// If external mem is not used, the word clock is the same as
	// internal max clock. Do nothing
	always @(posedge defaultMAXclock) begin
	  if (!xMemEnable) begin
	  end
	  else if (reset) begin
			// goes back to the 0 state
			currentState <= 0;
			currentByteCount <= 0;
			// reset all the bytes
			eepromByte0 <= 0;
			eepromByte1 <= 0;
			eepromByte2 <= 0;
			eepromByte3 <= 0;
		end
	  else begin
			// starts the state machine
			case (currentState)

				 // ROM OPERATION
				 // Read 1st byte
				 0:  begin
					  wordClk <= 1;
					  
					  AS <= 0;        // disable the address strobe
					  DS <= 0;        // disable the data strobe
					  MemWrite <= 0;  // disable memWrite
					  ROM <= 1;       // select the ROM
					  RAM <= 0;			// deselect the RAM
					  
				 end

				 // state 1: enable address strobe
				 1:  AS <= 1;

				 // state 2: enable data strobe
				 2:  DS <= 1;

				 // 50 MHz = 20 ns period
				 // 10 states = 200 ns (state 3 -> 12)

				 // state 13: capture the data and store in 1st byte
				 13: eepromByte0 <= dataIn;

				 // state 14: disable the datastrobe
				 14: DS <= 0;

				 // state 15: Prepare to read 2nd byte
				 15: begin
					  AS <= 0;
					  currentByteCount <= currentByteCount + 1'b1;
				 end 

				 // state 16: enable address strobe
				 16:  AS <= 1;

				 // state 17: enable data strobe
				 17:  DS <= 1;

				 // 50 MHz = 20 ns period
				 // 10 states = 200 ns (state 18 -> 27)

				 // state 28: capture the data and store in 1st byte
				 28: eepromByte1 <= dataIn;

				 // state 29: disable the datastrobe
				 29: DS <= 0;

				 // state 30: Prepare to read 3rd byte
				 30: begin
					  AS <= 0;
					  currentByteCount <= currentByteCount + 1'b1;
				 end

				 // state 31: enable address strobe
				 31:  AS <= 1;

				 // state 32: enable data strobe
				 32:  DS <= 1;

				 // 50 MHz = 20 ns period
				 // 10 states = 200 ns (state 33 -> 42)

				 // state 43: capture the data and store in 1st byte
				 43: eepromByte2 <= dataIn;

				 // state 44: disable the datastrobe
				 44: DS <= 0;

				 // state 45: Prepare to read 4th byte
				 45: begin
					  AS <= 0;
					  currentByteCount <= currentByteCount + 1'b1;
				 end

				 // state 1: enable address strobe
				 46:  AS <= 1;

				 // state 2: enable data strobe
				 47:  DS <= 1;

				 // 50 MHz = 20 ns period
				 // 10 states = 200 ns (state 48 -> 57)

				 // state 13: capture the data and store in 1st byte
				 58: eepromByte3 <= dataIn;

				 // state 14: end ROM read cycle
				 59: begin
					  // flip the word clock edge to 0
					  wordClk <= ~wordClk;

					  // disable address & data strobes, and deselect ROM
					  // Recall that 0 is the logic false, not the pin state.
					  // Although ROM is active low, the logic is inverted
					  // when assigned to GPIO.
					  AS <= 0;
					  DS <= 0;
					  ROM <= 0;

					  // assemble the word
					  // and send it out of the module
					  Instr <= {eepromByte3, eepromByte2, eepromByte1, eepromByte0}; // little endian

					  // reset the byte count
					  currentByteCount <= 0;
				 end

				 // SRAM operation
				 // load = read from dmem (sram)
				 // store = write to dmem (sram)
				 60: begin
					  if (isLoad) begin
							// disable write to memory
							MemWrite <= 0;
							
							// enable the RAM
							RAM <= 1;

					  end
					  else if (isStore) begin
							// enable write to memory
							MemWrite <= 1;
							
							// enable the RAM
							RAM <= 1;
							
							// write 1st higher order byte
							dataBus <= WriteData[31:24];
					  end
				 end

				 // state 61: enable address strobe
				 61: begin
					  if (isSramOp) AS <= 1;
				 end

				 // state 62: enable the data strobe
				 62: begin
					  if (isSramOp) DS <= 1;
				 end

				 // state 63 -> 72 = 10 states
				 // 20 ns * 10 = 200 ns

				 // state 73: retrives the 1st byte (if in read mode)
				 73: begin
					  if(isLoad)
							sramByte0 <= dataIn;
				 end

				 // state 74: disable the data strobe
				 74: DS <= 0;

				 // state 75: prepare to read/write 2nd byte
				 75: begin
					  AS <= 0;

					  if (isSramOp) begin
							currentByteCount <= currentByteCount + 1'b1;

							// if store, put the data out as well
							if (isStore)
								 dataBus <= WriteData[23:16];
					  end
				 end

				 // state 76: enable address strobe
				 76: begin
					  if (isSramOp) AS <= 1;
				 end

				 // state 77: enable the data strobe
				 77: begin
					  if (isSramOp) DS <= 1;
				 end

				 // state 78 -> 87 = 10 states
				 // 20 ns * 10 = 200 ns

				 // state 88: retrives the 1st byte (if in read mode)
				 88: begin
					  if(isLoad)
							sramByte1 <= dataIn;
				 end

				 // state 89: disable the data strobe
				 89: DS <= 0;

				 // state 90: prepare to read/write 3rd byte
				 90: begin
					  AS <= 0;

					  if (isSramOp) begin
							currentByteCount <= currentByteCount + 1'b1;

							// if store, put the data out as well
							if (isStore)
								 dataBus <= WriteData[15:8];
					  end
				 end

				 // state 91: enable address strobe
				 91: begin
					  if (isSramOp) AS <= 1;
				 end

				 // state 92: enable the data strobe
				 92: begin
					  if (isSramOp) DS <= 1;
				 end

				 // state 93 -> 102 = 10 states
				 // 20 ns * 10 = 200 ns

				 // state 103: retrives the 1st byte (if in read mode)
				 103: begin
					  if(isLoad)
							sramByte2 <= dataIn;
				 end

				 // state 104: disable the data strobe
				 104: DS <= 0;

				 // state 105: prepare to read/write 4th byte
				 105: begin
					  AS <= 0;

					  if (isSramOp) begin
							currentByteCount <= currentByteCount + 1'b1;

							// if store, put the data out as well
							if (isStore)
								 dataBus <= WriteData[7:0];
					  end
				 end

				 // state 106: enable address strobe
				 106: begin
					  if (isSramOp) AS <= 1;
				 end

				 // state 107: enable the data strobe
				 107: begin
					  if (isSramOp) DS <= 1;
				 end

				 // state 108 -> 117 = 10 states
				 // 20 ns * 10 = 200 ns

				 // state 118: retrives the 1st byte (if in read mode)
				 118: begin
					  if(isLoad)
							sramByte3 <= dataIn;
				 end

				 // state 119: disable the data strobe
				 119: DS <= 0;

				 // state 120: ends Memory read/write cycle
				 120: begin
					  AS <= 0;

					  if (isSramOp) begin

							// if store, do nothing, because all
							// of the bytes have been written
							// if load, then output to ReadData variable
							if (isLoad)
								 ReadData <= {sramByte0, sramByte1, sramByte2, sramByte3};
					  end
					  
					  // reset the byte count & state
					  currentByteCount <= 0;
					  currentState <= 0;
					  
					  // reset all the bytes, so that state machine
					  // reset to state 0 will get address mux from
					  // eeprom instead of sram
					  eepromByte0 <= 0;
					  eepromByte1 <= 0;
					  eepromByte2 <= 0;
					  eepromByte3 <= 0;
					  
					  // disable the RAM
					  RAM <= 0;
				 end
			endcase
			
			// increments to the next state
			if (currentState < 120)
				 currentState <= currentState + 1'b1;

		end
	end

endmodule : xmem