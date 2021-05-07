module hazard_detect_unit (input [4:0] if_id_rs, input [4:0] if_id_rt, input [4:0] id_ex_rt,input id_ex_mem_read, 
			   output reg control_line, output reg  if_id_write, output reg pc_write);

always@(if_id_rs, if_id_rt, id_ex_rt, id_ex_mem_read)
begin
	if(id_ex_mem_read)	//if hazard occurs make the control lines = 0  
	begin
		if(id_ex_rt == if_id_rs || id_ex_rt == if_id_rt)
		begin
			control_line = 1'b0;
			if_id_write = 1'b0;
			pc_write = 1'b0;
		end
	end
	else					//if hazard doesnot occurs make the control lines = 1 
	begin
			control_line = 1'b1;
			if_id_write = 1'b1;
			pc_write = 1'b1;
	end
end
endmodule
