
module pc_update (input [31:0] pc , input [31:0] sign_ext_offset , 
		output reg [31:0] pc_4 , output reg [31:0] pc_4_offset  );
	
	always @ (pc ,sign_ext_offset) begin
	pc_4 = pc + 32'd1;
	pc_4_offset  = pc + 32'd1 + sign_ext_offset; 
	end
endmodule 