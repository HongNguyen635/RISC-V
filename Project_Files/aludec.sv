module aludec (
	input 	logic			opb5,
	input		logic [2:0] funct3,
	input		logic			funct7b5,
	input		logic [1:0] ALUOp,
	output	logic [3:0] ALUControl);
	
	logic RtypeSub;
	assign RtypeSub = funct7b5 & opb5; // True for R-type subtract
	
	always_comb
		// determine if addition or subtraction
		case(ALUOp)
			2'b00: ALUControl = 4'b0000; // addition
			2'b01: ALUControl = 4'b0001; // subtraction
			
			default: 
				// then check for the instruction type
				case(funct3) // R–type or I–type ALU
					3'b000: if (RtypeSub)
									ALUControl = 4'b0001; // sub
							  else
									ALUControl = 4'b0000; // add, addi
					3'b010: ALUControl = 4'b0101; // slt, slti
					3'b100: ALUControl = 4'b0100;  // xor, xori
					3'b110: ALUControl = 4'b0011; // or, ori
					3'b111: ALUControl = 4'b0010; // and, andi
					3'b001: if (opb5) // opb5 means register operation
									ALUControl = 4'b1001; // sll
								else
									ALUControl = 4'b0110; // slli
					3'b101: if (funct7b5) begin
									if (opb5)
										ALUControl = 4'b1011; // sra
									else
										ALUControl = 4'b1000; // srai
							  end else begin
									if (opb5)
										ALUControl = 4'b1010; // srl
									else
										ALUControl = 4'b0111; // srli
								end
					default: ALUControl = 4'bxxxx; // ???
				endcase
			endcase
	
endmodule