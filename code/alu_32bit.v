module alu_32bit(input[31:0] in1 ,input [31:0] in2 ,input [3:0] aluCtrl, output reg [31:0] result , output reg zero );

	always@(in1,in2,aluCtrl) begin
	case(aluCtrl)
	4'b0000 : result = in1 & in2;
	4'b0001 : result = in1 | in2;
	4'b0010 : result = in1 + in2;
	4'b0110 : result = in1 - in2;
	4'b0111 : result = in1 < in2 ? 32'b00000000000000000000000000000001 : 32'b0;
	4'b1100 : result = ~(in1 | in2) ;
	endcase
	
	if(result == 0) 
		zero = 1'b1;
	else 
		zero = 1'b0; 

	

	end

endmodule

module simple_alu_32bit(input[31:0] in1 ,input [31:0] in2 , output reg [31:0] result );

	always@(in1,in2) begin
	
	result = in1+in2;
	
	end

endmodule

