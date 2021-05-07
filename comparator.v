module compare_hazard(input [31:0] read_data1 , input [31:0] read_data2 , input branch , output reg if_id_flush , output reg pc_src);
	always@(read_data1 ,read_data2 , branch) begin 
	if (read_data1 == read_data2 && branch) begin 
		if_id_flush <= 1'b1;
		pc_src <= 1'b1;
	end else begin 
		if_id_flush <= 1'b0;
		pc_src <= 1'b0;
		end 
	end
endmodule
