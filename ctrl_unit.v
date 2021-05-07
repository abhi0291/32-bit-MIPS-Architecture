module ctrl_unit(input [5:0] opcode , output reg [1:0] aluOp , output reg regDst , output reg aluSrc , output reg memToReg , 
				      output reg regWrite , output reg memWrite , output reg memRead , output reg branch );
	always @(opcode)
	begin
	case (opcode) 

	6'b000000 : begin //R-type
		aluOp  = 2'b10;
		regDst = 1'b1;
		aluSrc = 1'b0;
		memToReg = 1'b0;
		regWrite = 1'b1;
		memWrite = 1'b0;
		memRead = 1'b0;
		branch = 1'b0; 
		end
	6'b100011 : begin //lw
		aluOp  = 2'b00;
		regDst = 1'b0;
		aluSrc = 1'b1;
		memToReg = 1'b1;
		regWrite = 1'b1;
		memWrite = 1'b0;
		memRead = 1'b1;
		branch = 1'b0;
		end
	6'b101011 : begin //sw
		aluOp  = 2'b00;
		regDst = 1'b1;
		aluSrc = 1'b1;
		memToReg = 1'b0;
		regWrite = 1'b0;
		memWrite = 1'b1;
		memRead = 1'b0;
		branch = 1'b0;
		end
	6'b000100 : begin //beq
		aluOp  = 2'b10;
		regDst = 1'b1;
		aluSrc = 1'b0;
		memToReg = 1'b0;
		regWrite = 1'b0;
		memWrite = 1'b0;
		memRead = 1'b0;
		branch = 1'b1;
		end
	6'd8 : begin //adi
		aluOp  = 2'b10;
		regDst = 1'b0;
		aluSrc = 1'b0;
		memToReg = 1'b0;
		regWrite = 1'b1;
		memWrite = 1'b0;
		memRead = 1'b0;
		branch = 1'b0; 
		end
	endcase
	end

endmodule


