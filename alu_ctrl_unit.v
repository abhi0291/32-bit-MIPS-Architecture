module alu_ctrl_unit(input [1:0] alu_op , input [5:0] func, output reg [3:0] alu_ctrl);
	
	always @ (alu_op,func) begin
	
	case(alu_op)
  		2'b00 : alu_ctrl = 4'b0010;
		2'b01 : alu_ctrl = 4'b0110;
		2'b10 : 
			if(func == 6'b100000) 
				alu_ctrl = 4'b0010;
			else if(func == 6'b100010) 
				alu_ctrl = 4'b0110;
			else if(func == 6'b100100) 
				alu_ctrl = 4'b0000;
			else if(func == 6'b100101) 
				alu_ctrl = 4'b0001;
			else if(func == 6'b101010) 
				alu_ctrl = 4'b0111;
			else if (func == 6'b100111)
				alu_ctrl = 4'b1100;
			else alu_ctrl = 4'bxxxx;
		default : alu_ctrl = 4'bxxxx;
		endcase
	end

endmodule
