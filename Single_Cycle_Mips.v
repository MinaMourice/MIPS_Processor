/*
5 Stage Single MIPS Processor 
Created By : ASU_3rd_Year_Student
Team Member :
1- Verina Alber
	Instruction Memory 
2- Marina Saad
	Registers File
3- Mina Magdy
	ControlUnit
4- Mina Mourice
	ALU 
	ALUControl
5- Hnaa Adel
	MUX
*/


/*
---------------------------------------------------------------------------------------------------------------------------------
Instruction Memory
Verina Alber
---------------------------------------------------------------------------------------------------------------------------------

*/


`ifndef _alu_control
`define _alu_control
module InstructMem(PC,Inst); 
    input [31:0] PC; 
    output [31:0] Inst; 
     
    reg [31:0] regfile[32:0];//32 32-bit register 
     initial

	begin

	$readmemb("Instructions.txt",regfile,0,31);
	//#1
//	$display("%b",regfile[40]);

	end
    assign Inst = regfile[PC]; //assigns output to instruction 
     
endmodule 
/*

module InstructMem(PC,Inst); 
    input [31:0] PC; 
    output [31:0] Inst; 
     
    reg [31:0] regfile[199:0];//32 32-bit register 
     always@(PC)

	begin

	$readmemb("Instructions.txt",regfile);
	//#1
//	$display("%b",regfile[40]);

	end

    assign Inst = regfile[PC]; //assigns output to instruction 
     
endmodule */
/*
module TEST_instru ;

reg   [31:0]  Read_address;

wire 	[31:0] 	Instruction;
reg clk=0;

InstructMem test(Read_address,Instruction);



initial

begin

  $monitor("%d %b " ,Read_address,Instruction);
#20
Read_address <=40 ;
#20
Read_address <=50 ;
#20
Read_address <=60 ;
end
always
begin
#10 clk = ~clk;
end

endmodule
*/

/*
---------------------------------------------------------------------------------------------------------------------------------
Register File
Marina Saad
---------------------------------------------------------------------------------------------------------------------------------

*/



module Registers(clock,WE,InData,WrReg,ReadA,ReadB,OutA,OutB); 
    input [4:0] WrReg, ReadA, ReadB; 
    input WE,clock; 
    input [31:0] InData; 
    output [31:0] OutA,OutB; 
     
    reg [31:0] OutA, OutB;//2 32-bit output reg 
    reg [31:0] regfile[511:0];//32 32-bit registers 
     
    initial begin 
        OutA = -20572; //random values for initial 
        OutB = -398567; 
    end 
     
    always@(clock,InData,WrReg,WE) 
    begin 
      if(WE && clock) 
        begin 
         regfile[WrReg]<=InData;//write to register 
         $display("Does WrReg: %d Data: %d",WrReg,InData); 
        end 

    end 
     
    always @ (clock,ReadA,ReadB,WrReg) 
    begin 
        if(~clock) 
        begin 
      OutA <= regfile[ReadA];//read values from registers 
      OutB <= regfile[ReadB]; 
      $monitor  ("R0:  %d  R1:  %d  R2:  %d \nR3:  %d  R4:  %d  R5:  %d \nR6:  %d  R7:  %d  R8:  %d \nR9:  %d  R10:  %d R11:  %d \nR12:  %d  R13:  %d  R14:  %d \nR15:  %d  R16:  %d  R17:  %d \nR18:  %d  R19:  %d  R20:  %d \nR21:  %d  R22:  %d  R23:  %d \nR24:  %d  R25:  %d  R26:  %d \nR27:  %d  R28:  %d  R29:  %d \nR30:  %d R31:  %d\n",
regfile[0],regfile[1],regfile[2],regfile[3],regfile[4],regfile[5],regfile[6],regfile[7],regfile[8],regfile[9],regfile[10]
,regfile[11],regfile[12],regfile[13],regfile[14],regfile[15],regfile[16],regfile[17],regfile[18],regfile[19],regfile[20]
,regfile[21],regfile[22],regfile[23],regfile[24],regfile[25],regfile[26],regfile[27],regfile[28],regfile[29],regfile[30],regfile[31]); 
      end 
    end 
 
 
endmodule 



/*
---------------------------------------------------------------------------------------------------------------------------------
ALU
Mina Magdy
---------------------------------------------------------------------------------------------------------------------------------

*/

module ALU(ALUCon,DataA,DataB,Result); 
    input [3:0] ALUCon; 
    input [31:0] DataA,DataB; 
    output [31:0] Result; 
     
    reg [31:0] Result; 
    reg Zero; 
     
    initial begin 
        Result = 32'd0; 
    end 
 
    always@(ALUCon,DataA,DataB) 
    begin  
      case(ALUCon) 
          4'b0000://and 
		Result <= DataA&DataB;
          4'b0001://or 
             	Result <= DataA|DataB; 
          4'b0010://add 
              	Result <= DataA+DataB; 
          4'b0011://multiply 
             	Result <= DataA*DataB; 
          4'b0100://nor 
		Result <= ~(DataA|DataB);
          4'b0101://divide 
		Result <= DataA/DataB; 
          4'b0110://sub 
		Result <= DataA-DataB; 
          4'b0111://slt 
		Result = DataA<DataB ? 1:0; 
          4'b1000://sll 
		Result <= (DataA<<DataB); 
          4'b0110://srl 
		Result <= (DataA>>DataB);     

          default: //error 
          begin 
              $display("ALUERROR"); 
              Result = 0; 
          end 
           
      endcase 
	$display("Result=%d",Result); 
    end 
 
endmodule 



/*
---------------------------------------------------------------------------------------------------------------------------------
ALU Control
Mina Mourice
---------------------------------------------------------------------------------------------------------------------------------

*/

module ALUControl(andi,ori,addi,ALUOp,funct,ALUCon); 
    input [1:0] ALUOp; 
    input [5:0] funct; 
    input andi,ori,addi; 
    output [3:0] ALUCon; 
     
    reg [3:0] ALUCon; 
     
    always@(ALUOp or funct or andi or ori or addi) 
    begin 
      case(ALUOp) 
        2'b00://lw or sw 
           ALUCon = 4'b0010; 
         
        2'b01://beq 
           ALUCon = 4'b0110; 
         
        2'b10://R-type 
        begin 
           if(funct==6'b100100) 
              ALUCon = 4'b0000;//and 
           if(funct==6'b100101) 
              ALUCon = 4'b0001;//or 
           if(funct==6'b100000) 
              ALUCon = 4'b0010;//add 
           if(funct==6'b011000) 
              ALUCon = 4'b0011;//multi 
           if(funct==6'b100111) 
              ALUCon = 4'b0100;//nor 
           if(funct==6'b011010) 
              ALUCon = 4'b0101;//div 
           if(funct==6'b100010) 
              ALUCon = 4'b0110;//sub 
           if(funct==6'b101010) 
              ALUCon = 4'b0111;//slt 
        end 
      2'b11://immediate 
      begin 
          if(andi)begin 
             ALUCon = 4'b0000;//andi 
         end 
          if(ori) begin 
             ALUCon = 4'b0001;//ori 
         end 
          if(addi) 
             ALUCon = 4'b0010;//addi 
      end 
    endcase 
    end 
 
         
     
endmodule 


/*
---------------------------------------------------------------------------------------------------------------------------------
Control Unit For Single & Pipelined
Mina Mourice
---------------------------------------------------------------------------------------------------------------------------------

*/





module Control(Op,Out,j,bne,imm,andi,ori,addi); 
   input [5:0] Op; 
   output[8:0] Out; 
   output j,bne,imm,andi,ori,addi; 
    
   wire regdst,alusrc,memtoreg,regwrite,memread,memwrite,branch; 
    
   //determines type of instruction 
   wire r = ~Op[5]&~Op[4]&~Op[3]&~Op[2]&~Op[1]&~Op[0]; 
  wire lw = Op[5]&~Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0]; 
  wire sw = Op[5]&~Op[4]&Op[3]&~Op[2]&Op[1]&Op[0]; 
  wire beq = ~Op[5]&~Op[4]&~Op[3]&Op[2]&~Op[1]&~Op[0]; 
  wire bne = ~Op[5]&~Op[4]&~Op[3]&Op[2]&~Op[1]&Op[0]; 
  wire j = ~Op[5]&~Op[4]&~Op[3]&~Op[2]&Op[1]&~Op[0]; 
   wire andi = ~Op[5]&~Op[4]&Op[3]&Op[2]&~Op[1]&~Op[0]; 
   wire ori = ~Op[5]&~Op[4]&Op[3]&Op[2]&~Op[1]&Op[0]; 
   wire addi = ~Op[5]&~Op[4]&Op[3]&~Op[2]&~Op[1]&~Op[0]; 
   wire imm = andi|ori|addi; //immediate value type 
    
   //seperate control arrays for reference 
   wire [3:0] EXE; 
   wire [2:0] M; 
   wire [1:0] WB; 
    
  // microcode control 
       assign regdst = r; 
  assign alusrc = lw|sw|imm; 
  assign memtoreg = lw; 
  assign regwrite = r|lw|imm; 
  assign memread = lw; 
  assign memwrite = sw; 
  assign branch = beq; 
   
  // EXE control 
  assign EXE[3] = regdst; 
  assign EXE[2] = alusrc; 
  assign EXE[1] = r; 
  assign EXE[0] = beq; 
       
  //M control 
  assign M[2] = branch; 
  assign M[1] = memread; 
  assign M[0] = memwrite; 
   
  //WB control 
  assign WB[1] = memtoreg; //not same as diagram 
  assign WB[0] = regwrite; 
   
  //output control 
  assign Out[8:7] = WB; 
  assign Out[6:4] = M; 
  assign Out[3:0] = EXE; 
   
endmodule 
 
 
/*
---------------------------------------------------------------------------------------------------------------------------------
Data Memory
Hanaa Adel
---------------------------------------------------------------------------------------------------------------------------------

*/


module DATAMEM(MemWrite,MemRead,Addr,Wdata,Rdata); 
    input [31:0] Addr,Wdata; 
    input MemWrite,MemRead; 
    output [31:0] Rdata; 
     
    reg [31:0] Rdata; 
    reg [31:0] regfile[511:0];//32 32-bit registers 
   
 
    always@(Addr,Wdata,MemWrite,MemRead) 
    if(MemWrite) 
    begin 
      $display("Writing %d -> Addr: %d",Wdata,Addr); 
      regfile[Addr]<=Wdata; //memory write 
    end 
 
    always@(Addr,Wdata,MemWrite,MemRead) 
    if(MemRead) 
      Rdata <= regfile[Addr];//memory read 
 
endmodule 







/*
---------------------------------------------------------------------------------------------------------------------------------
Muliplixer
Hanaa Adel
---------------------------------------------------------------------------------------------------------------------------------

*/


module BIGMUX2(A,X0,X1,X2,X3,Out);//non-clocked mux 
input [1:0] A; 
input [31:0] X3,X2,X1,X0; 
output [31:0] Out; 
 
reg [31:0] Out; 
 
always@(A,X3,X2,X1,X0) 
begin 
  case(A) 
      2'b00: 
        Out <= X0; 
      2'b01: 
        Out <= X1; 
      2'b10: 
        Out <= X2; 
      2'b11: 
        Out <= X3; 
  endcase 
end 
 
endmodule 
 
`endif