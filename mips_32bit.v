
module mips_32bit(input clk, input reset);
	wire [31:0]  pc_next;
	reg [31:0] pc;
	wire [31:0]instr;
	wire [5:0] opcode = instr [31:26]; //Op
	wire [4:0] r_s = instr [25:21];	//Rs
	wire [4:0] r_t = instr [20:16]; // Rt
	wire [4:0] r_d = instr [15:11]; // Rd
	wire [15:0] offset = instr [15:0]; //Offset
	wire [5:0] func = instr [5:0]; //Function
	wire [1:0] alu_op;
	wire reg_dst , alu_src , mem_to_reg , reg_write , mem_write , mem_read , branch , pc_src;
	wire [4:0] temp1;
	wire [31:0] read_data1 , read_data2 , temp4; 
	wire [31:0] sign_ext_offset;
	wire [31:0] temp2;
	wire [3:0] alu_ctrl;
	wire [31:0] alu_result;
	wire zero;
	wire [31:0] read_data; 
	wire [31:0] pc_4 , pc_4_offset ;
	wire [31:0] temp3;
	
	always @ (posedge clk) begin 
		if(reset == 1)
			pc=32'b0;
		else pc = pc_next; 
	end

	// Instruction Fetch
	
	instr_mem mem1 (pc , instr);
	
	// Instruction Decode
	
	ctrl_unit maincontrol (opcode , alu_op ,  reg_dst , alu_src , mem_to_reg , reg_write , mem_write , mem_read , branch ) ;
	
	mux_5bit mux5_1 (r_t , r_d , reg_dst , temp1); 
	
	registors_file regfile (clk , reg_write ,  r_s , r_t , temp1 , temp4 , read_data1 , read_data2 );
	
	sign_extend se (offset , sign_ext_offset);
	
	//Execution
	
	mux_32bit mux32_1(read_data2 , sign_ext_offset , alu_src , temp2);
	
	alu_ctrl_unit aluctrl (alu_op , func , alu_ctrl);
	
	alu_32bit alu1 (read_data1 , temp2 , alu_ctrl , alu_result , zero);

	assign pc_src = zero & branch;
	
	data_mem mem2 (alu_result , read_data2 , mem_read , mem_write, read_data);

	mux_32bit mux32_2 (alu_result ,read_data ,  mem_to_reg , temp4) ; 
	 
	pc_update pcu ( pc , sign_ext_offset , pc_4 , pc_4_offset );
	
	mux_32bit mux32_3 (pc_4 , pc_4_offset , pc_src , pc_next);
	



endmodule