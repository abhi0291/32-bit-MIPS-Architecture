
//4.51, 4.56, 4.57, 4.60, 4.65, 4.66
module pipeline_mips_32bit(input clk, input reset);

	wire [31:0]  pc_next;
	reg [31:0] pc;
	wire [31:0]instr;
	wire [10:0] temp; 
	wire [1:0] alu_op , forwardA , forwardB;
	wire reg_dst , alu_src , mem_to_reg , reg_write , mem_write , mem_read , branch ,control_line, if_id_write,  pc_write,if_id_flush;
	wire [4:0] dst_reg;
	wire [31:0] read_data1 , read_data2 , temp4; 
	wire [31:0] sign_ext_offset;
	wire [31:0] temp2 , tempA , tempB;
	wire [3:0] alu_ctrl;
	wire [31:0] alu_result;
	wire zero;
	wire [31:0] read_data; 
	wire [31:0] pc_1 , pc_1_offset ;
	wire [31:0] temp3,result;
	wire pc_src;
	
	// IF/ID
	reg [31:0] if_id_pc_1;
	reg [5:0] if_id_opcode; 
	reg [4:0] if_id_rs , if_id_rt , if_id_rd ;
	reg [15:0] if_id_offset; 

	// ID/EXE
	reg [1:0] id_ex_wb; //0 = regwrite 1 = memtoreg
	reg [2:0] id_ex_m; // 0 = memread 1 = memwrite 2 = branch
	reg [5:0] id_ex_ex; //3:0 = aluop 4 = regdst 5 = alusrc
	reg [31:0] id_ex_read_data1, id_ex_read_data2, id_ex_sign_ext_offset, id_ex_pc_1;
	reg [4:0] id_ex_rs, id_ex_rt, id_ex_rd;

	// EXE/MEM
	reg [1:0] ex_mem_wb; //0 = regwrite 1 = memtoreg
	reg [2:0] ex_mem_m; // 0 = memread 1 = memwrite 2 = branch
	reg [31:0] ex_mem_pc_1_offset , ex_mem_alu_result , ex_mem_read_data2;
	reg ex_mem_zero;
	reg [4:0] ex_mem_dst_reg;
	
	// MEM/WB
	reg [1:0] mem_wb_wb; //0 = regwrite 1 = memtoreg
	reg [31:0] mem_wb_alu_result, mem_wb_read_data;
	reg [4:0] mem_wb_dst_reg;
	
	// Instruction Fetch
	mux_32x2 mux32_1 (pc_1 , pc_1_offset , pc_src , pc_next);
	
	always @ (posedge clk) begin 
		if(reset == 1)
			pc <= 32'b0;
		else begin
			if (pc_write) 
				pc <= pc_next;
			else pc <= pc;
		end
	end
	
	simple_alu_32bit s1 (pc , 32'b00000000000000000000000000000001 ,pc_1);
	
	instr_mem mem1 (pc , instr);
	
	// Instruction Decode
	always @(posedge clk) begin 
	if(if_id_flush == 0) begin
		if(if_id_write) begin 
			if_id_pc_1 <= pc_1;
			if_id_opcode <= instr [31:26];   //Op
			if_id_rs <= instr [25:21];      //Rs
			if_id_rt <= instr[20:16];       //Rt
			if_id_rd <= instr [15:11];      //Rd
			if_id_offset <= instr [15:0];    //Offset
		end else begin 
			if_id_pc_1 <= if_id_pc_1;
			if_id_opcode <= if_id_opcode;   //Op
			if_id_rs <= if_id_rs;      //Rs
			if_id_rt <= if_id_rt;       //Rt
			if_id_rd <= if_id_rd;      //Rd
			if_id_offset <= if_id_offset;    //Offset
		end
	end else begin
		if_id_pc_1 <= 32'b0;
		if_id_opcode <= 6'b0;   //Op
		if_id_rs <= 5'b0;      //Rs
		if_id_rt <= 5'b0;       //Rt
		if_id_rd <= 5'b0;      //Rd
		if_id_offset <= 16'b0;    //Offset
		end
	end

	
	hazard_detect_unit hdu (if_id_rs, if_id_rt, id_ex_rt , id_ex_m[0], control_line, if_id_write,  pc_write);
	ctrl_unit mcu (if_id_opcode , alu_op ,  reg_dst , alu_src , mem_to_reg , reg_write , mem_write , mem_read , branch ) ; 
	mux_11x2 mux11_1 (9'b0 , {mem_to_reg , reg_write , branch , mem_write , mem_read ,alu_src , reg_dst , alu_op[1:0]} , control_line , temp);
	registers_file regfile ( mem_wb_wb[0] ,  if_id_rs , if_id_rt , mem_wb_dst_reg , result , read_data1 , read_data2 );
	sign_extend se (if_id_offset , sign_ext_offset);
	simple_alu_32bit s2 (if_id_pc_1 , sign_ext_offset ,pc_1_offset);
	compare_hazard ch (read_data1 , read_data2 , branch ,  if_id_flush , pc_src); 
	
	//Execution
	always @(posedge clk) begin

	id_ex_wb <= temp[8:7]; //7 = regwrite, 8 = memtoreg
	id_ex_m <= temp[6:4]; // 4 = memread, 5 = memwrite, 6 = branch
	id_ex_ex <= temp[3:0]; //1:0 = aluop, 2 = regdst, 3 = alusrc
	id_ex_read_data1 <= read_data1;
	id_ex_read_data2 <= read_data2;
	id_ex_sign_ext_offset <= sign_ext_offset;
	id_ex_rs <= if_id_rs;
	id_ex_rt <= if_id_rt;
	id_ex_rd <= if_id_rd;

	end
	
	forwarding_unit fu (id_ex_rs , id_ex_rt , ex_mem_dst_reg , mem_wb_dst_reg , ex_mem_wb[0] , mem_wb_wb[0] , forwardA , forwardB );
	mux_32x3 mux32_2 (id_ex_read_data1 , result , ex_mem_alu_result , forwardA , tempA);
	mux_32x3 mux32_3 (id_ex_read_data2 , result , ex_mem_alu_result , forwardB , tempB);
	mux_5x2 mux5_1 (id_ex_rt , id_ex_rd , id_ex_ex[2] , dst_reg);
	mux_32x2 mux32_4(tempB , id_ex_sign_ext_offset , id_ex_ex[3] , temp2);
	alu_ctrl_unit ac (id_ex_ex[1:0] , id_ex_sign_ext_offset[5:0] , alu_ctrl);
	alu_32bit alu1 (tempA , temp2 , alu_ctrl , alu_result , zero);
	
	//Memory
	always @(posedge clk) begin
	
	ex_mem_wb <= id_ex_wb; //0 = regwrite, 1 = memtoreg
	ex_mem_m <= id_ex_m;   //0 = memread, 1 = memwrite, 2 = branch
	ex_mem_zero <= zero;
	ex_mem_alu_result <= alu_result;
	ex_mem_read_data2 <= tempB;
	ex_mem_dst_reg <= dst_reg;

	end

	data_mem mem2 (ex_mem_alu_result , ex_mem_read_data2 , ex_mem_m[0] , ex_mem_m[1] , read_data);
	
	//Write Back
	always @(posedge clk) begin

	mem_wb_wb <= ex_mem_wb; //0 = regwrite 1 = memtoreg
	mem_wb_alu_result <= ex_mem_alu_result;
	mem_wb_read_data <= read_data;
	mem_wb_dst_reg <= ex_mem_dst_reg;

	end

	mux_32x2 mux32_5 (mem_wb_alu_result ,mem_wb_read_data , mem_wb_wb[1] , result ) ; 	

endmodule
