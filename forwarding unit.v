module forwarding_unit (input [4:0] id_ex_rs , input [4:0] id_ex_rt , input [4:0] ex_mem_rd , input [4:0] mem_wb_rd , input ex_mem_reg_write , input mem_wb_reg_write ,
			output reg [1:0] forwardA , output reg [1:0] forwardB );

always @ (id_ex_rs , id_ex_rt , ex_mem_rd , mem_wb_rd , ex_mem_reg_write , mem_wb_reg_write )
	begin

	if (ex_mem_reg_write && ex_mem_rd != 0 && ex_mem_rd == id_ex_rs)
		begin 
			forwardA <= 2'b10;
		end
	else if (mem_wb_reg_write && mem_wb_rd != 0 && ~(ex_mem_reg_write && ex_mem_rd != 0 && ex_mem_rd != id_ex_rs) && mem_wb_rd == id_ex_rs)
		begin 
			forwardA <= 2'b01;
		end
	else forwardA <= 2'b00;

	if (ex_mem_reg_write && ex_mem_rd != 0 && ex_mem_rd == id_ex_rt)
		begin 
			forwardB <= 2'b10;
		end
	
	else if (mem_wb_reg_write && mem_wb_rd != 0 && ~(ex_mem_reg_write && ex_mem_rd != 0 && ex_mem_rd != id_ex_rs) &&  mem_wb_rd == id_ex_rt)
		begin 
			forwardB <= 2'b01;
		end

	else forwardB <= 2'b00;
	
	end

endmodule

