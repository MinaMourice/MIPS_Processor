module Instruction_Memory (Read_address,Instruction);

input  [31:0] Read_address;
output [31:0] Instruction;
reg [31:0] Memory [31:0];
initial 
begin
$readmemh("C:/Modeltech_pe_edu_10.4a/examples/program.mips.txt",Memory);
end
assign Instruction = Memory [Read_address];
endmodule

module TEST ;
reg   [31:0]  Read_address;
wire 	[31:0] 	Instruction;
Instruction_Memory test(Read_address,Instruction);

initial
begin
$monitor("%d %h " ,Read_address,Instruction);
Read_address <=1 ;
end
endmodule
