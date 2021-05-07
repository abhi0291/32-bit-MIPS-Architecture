//Instruction Memory
module instr_mem(input [31:0] addr, output [31:0] instr); 
	reg [32:0] rom[15:0];
	assign instr =  addr < 16 ? rom[addr[31:0]] : 32'b0 ; // provides required instruction
	initial 
		begin 
		
		rom[0] = 32'b00000001010010110100100000100010;  //sub t1 t2 t3;
        rom[1] = 32'b00010010101101100000000000000010; 	//beq s5 s6 0x2;
        rom[2] = 32'b00000001100011010101100000100000;  //add t3 t4 t5;
        rom[3] = 32'b00000001101011100110000000100000;  //add t4 t5 t6;
        rom[4] = 32'b00000001110011110110100000100000;  //add t5 t6 t7;
        rom[5] = 32'b10001110010100010000000000000000;	//lw s1 0x0 s2;
        rom[6] = 32'b00000000000000000000000000000000; 
        rom[7] = 32'b00000000000000000000000000000000;  
        rom[8] = 32'b00000000000000000000000000000000;  
        rom[9] = 32'b00000000000000000000000000000000; 
        rom[10] = 32'b00000000000000000000000000000000;  
        rom[11] = 32'b00000000000000000000000000000000;  
        rom[12] = 32'b00000000000000000000000000000000;
        rom[13] = 32'b00000000000000000000000000000000;  
        rom[14] = 32'b00000000000000000000000000000000;
		rom[15] = 32'b00000000000000000000000000000000;
		
		end 
	
endmodule

//Data Memory
module data_mem(input [31:0] addr, input [31:0] writeData, input memRead, input memWrite, output reg [31:0]readData);
	
	reg [32:0] ram [15:0];
	initial 
	begin 
			ram[0] = 32'b00000000000000000000000000000000;  
            ram[1] = 32'b00000000000000000000000000000001; 
            ram[2] = 32'b00000000000000000000000000000010;  
            ram[3] = 32'b00000000000000000000000000000011;  
            ram[4] = 32'b00000000000000000000000000000100;  
            ram[5] = 32'b00000000000000000000000000000101;
            ram[6] = 32'b00000000000000000000000000000111;  
            ram[7] = 32'b00000000000000000000000000001000; 
            ram[8] = 32'b00000000000000000000000000000000;  
            ram[9] = 32'b00000000000000000000000000000000; 
            ram[10] = 32'b00000000000000000000000000000000;  
            ram[11] = 32'b00000000000000000000000000000000;  
            ram[12] = 32'b00000000000000000000000000000000;
            ram[13] = 32'b00000000000000000000000000000000;  
            ram[14] = 32'b00000000000000000000000000000000;  
            ram[15] = 32'b00000000000000000000000000000000;
	end 
	always@ (addr,writeData,memRead,memWrite) begin
	
	 readData =  memRead ? ram[addr] : 32'b0 ;

	if (memWrite)
 		ram[addr] <= writeData;
	end
endmodule

//Register File
module registers_file ( input reg_write , input [4:0]  read_reg1 , input [4:0] read_reg2, input [4:0] write_reg , input [31:0] write_data
			, output reg [31:0] read_data1 , output reg [31:0] read_data2 );
	
	reg [31:0] registors[31:0];
	
	initial begin 

	registors[0] <= 32'b00000000000000000000000000000000; //$0
	registors[1] <= 32'b00000000000000000000000000000000;
	registors[2] <= 32'b00000000000000000000000000000000;
	registors[3] <= 32'b00000000000000000000000000000000;
	registors[4] <= 32'b00000000000000000000000000000000;
	registors[5] <= 32'b00000000000000000000000000000000;
	registors[6] <= 32'b00000000000000000000000000000000;
	registors[7] <= 32'b00000000000000000000000000000000;
	registors[8] <= 32'b00000000000000000000000000000000; //$t0
	registors[9] <= 32'b00000000000000000000000000000001;
	registors[10] <= 32'b00000000000000000000000000000010;
	registors[11] <= 32'b00000000000000000000000000000011;
	registors[12] <= 32'b00000000000000000000000000000100;
	registors[13] <= 32'b00000000000000000000000000000101;
	registors[14] <= 32'b00000000000000000000000000000110;
	registors[15] <= 32'b00000000000000000000000000000111; //$t7
	registors[16] <= 32'b00000000000000000000000000001000; //$s0
	registors[17] <= 32'b00000000000000000000000000001001;
	registors[18] <= 32'b00000000000000000000000000000001;
	registors[19] <= 32'b00000000000000000000000000001011;
	registors[20] <= 32'b00000000000000000000000000001100;
	registors[21] <= 32'b00000000000000000000000000001101;
	registors[22] <= 32'b00000000000000000000000000001101;
	registors[23] <= 32'b00000000000000000000000000001111; //$s7
	registors[24] <= 32'b00000000000000000000000000000000;
	registors[25] <= 32'b00000000000000000000000000000000;
	registors[26] <= 32'b00000000000000000000000000000000;
	registors[27] <= 32'b00000000000000000000000000000000;
	registors[28] <= 32'b00000000000000000000000000000000;
	registors[29] <= 32'b00000000000000000000000000000000;
	registors[30] <= 32'b00000000000000000000000000000000;
	registors[31] <= 32'b00000000000000000000000000000000;

	end 	

	always @( reg_write , read_reg1 , read_reg2 , write_reg , write_data ) begin
	if (reg_write) begin
		registors[write_reg] <= write_data;
		end
	read_data1 <= registors[read_reg1];
	read_data2 <= registors[read_reg2];
	end
			 
endmodule

// Main Control Unit
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

// Hazard Detection for LW
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

// Hazard Detection for BEQ
module compare_hazard(input [31:0] read_data1 , input [31:0] read_data2 , input branch , output reg if_id_flush , output reg pc_src);
	always@(read_data1 ,read_data2 , branch) begin 
	if (read_data1 == read_data2 && branch) //if hazard occurs make the control lines = 1
	begin
		if_id_flush <= 1'b1;
		pc_src <= 1'b1;
	end else begin ////if hazard dose not occurs make the control lines = 0
		if_id_flush <= 1'b0;
		pc_src <= 1'b0;
		end 
	end
endmodule

// All type of MUX which are required
module mux_5x2(input [4:0] in1 , input [4:0] in2  , input control ,output reg [4:0] out );
	
	always @ (in1 , in2 , control)
		out <= control ? in2 : in1;

endmodule
module mux_32x2(input [31:0] in1 , input [31:0] in2  , input control ,output reg [31:0] out);
	
	always @ (in1 , in2 , control)
		out <= control ? in2 : in1;

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
// mux ends

// Sign Extention
module sign_extend(input [15:0]in , output reg [31:0]out );

	always @(in) begin
		out [15:0]  <=  in[15:0];
		out [31:16] <= {16{in[15]}};
	end
	
endmodule

// Forwarding Unit
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

// Alu Control Unit
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

 // ALUs' required
module alu_32bit(input[31:0] in1 ,input [31:0] in2 ,input [3:0] aluCtrl, output reg [31:0] result , output reg zero );

	always@(in1,in2,aluCtrl) begin
	case(aluCtrl)
	4'b0000 : result = in1 & in2; //and
	4'b0001 : result = in1 | in2; //or
	4'b0010 : result = in1 + in2; //add
	4'b0110 : result = in1 - in2; //sub
	4'b0111 : result = in1 < in2 ? 32'b00000000000000000000000000000001 : 32'b0; //slt
	4'b1100 : result = ~(in1 | in2) ; //nor
	endcase
	
	if(result == 0) 
		zero = 1'b1;
	else 
		zero = 1'b0; 

	

	end

endmodule

module simple_alu_32bit(input[31:0] in1 ,input [31:0] in2 , output reg [31:0] result );
// performs only add
	always@(in1,in2) begin
	
	result = in1+in2;
	
	end

endmodule
//alu ends

//Main Module 
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

// Testbnch For Main Module 
module testbench;

reg clk;
reg reset;

pipeline_mips_32bit uut (clk,reset);

initial begin
clk = 0;
reset = 1;
forever #10 clk = ~ clk;
end

initial begin 
#20 reset = 0;
#300 $finish;
end 

endmodule










