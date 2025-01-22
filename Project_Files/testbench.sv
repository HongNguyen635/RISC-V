module testbench();
	logic clk;
	logic reset;
	logic [31:0] WriteData, DataAdr;
	logic MemWrite;
	
	// instantiate device to be tested
	// dut = device under test?
	topLevel dut(clk, reset, WriteData, DataAdr, MemWrite);
	
	// initialize test
	// initial = block that run once at the start of the sim
	// https://www.chipverify.com/verilog/verilog-initial-block
	initial
		begin
			// reset, wait for 22 sim time unit, then de-assert the reset
			reset <= 1; # 22; reset <= 0;
		end
		
		// generate clock to sequence tests
	always
		begin
			// generate the clock for 5 on and 5 off
			clk <= 1; # 5; clk <= 0; # 5;
		end
		
		// check results
		always @(negedge clk)
			begin
			
				if(MemWrite) begin
				if(DataAdr === 100 & WriteData === 25) begin
					$display("Simulation succeeded");
					$stop;
					
				end else if (DataAdr !== 96) begin
					$display("Simulation failed");
					$stop;
					
				end
			end
		end
endmodule