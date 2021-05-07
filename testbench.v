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
