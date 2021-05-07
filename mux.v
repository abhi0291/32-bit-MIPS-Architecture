module mux_5x2(input [4:0] in1 , input [4:0] in2  , input control ,output reg [4:0] out );
	
	always @ (in1 , in2 , control)
		out <= control ? in2 : in1;

endmodule
module mux_32x2(input [31:0] in1 , input [31:0] in2  , input control ,output reg [31:0] out);
	
	always @ (in1 , in2 , control)
		out <= control ? in2 : in1;

endmodule

module sign_extend(input [15:0]in , output reg [31:0]out );

	always @(in) begin
		out [15:0]  <=  in[15:0];
		out [31:16] <= {16{in[15]}};
	end
	
endmodule

module mux_32x3(input [31:0] in1 , input [31:0] in2  , input [31:0] in3 , input[1:0] control ,output reg [31:0] out ) ;
 
	always @ (in1 , in2, in3, control) begin
		if (control == 2'b00) out <= in1;
		else if (control == 2'b01) out <= in2;
		else if (control == 2'b10) out <= in3;
		else out <= 32'bx;
	end

endmodule 

module mux_11x2(input [8:0] in1 , input [8:0] in2  , input control ,output reg [10:0] out);
	
	always @ (in1 , in2 , control)
		out <= control ? in2 : in1;

endmodule