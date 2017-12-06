/*
5 Stage Pipeline MIPS Processor 
Created By : ASU_3rd_Year_Student
Team Member :
1- Verina Alber
	Instruction Fetch / Instruction Decode Register
		- IFID_Reg (IF_Flush,clk,IFID_Write,PC_4,Instruction,InstReg,PC_Plus4Reg); 

2- Marina Saad
	Instruction Decode / Instruction Execute Register
		- IDEX(clock,WB,M,EX,DataA,DataB,imm_value,RegRs,RegRt,RegRd,WBreg,Mreg,EXreg,DataAreg, 
			DataBreg,imm_valuereg,RegRsreg,RegRtreg,RegRdreg); 
3- Mina Magdy
	Instruction Execute / Memory Register
		- EXMEM(clock,WB,M,ALUOut,RegRD,WriteDataIn,Mreg,WBreg,ALUreg,RegRDreg,WriteDataOut);
	
4- Mina Mourice
	Hazard Unit
		-HazardUnit(IDRegRs,IDRegRt,EXRegRt,EXMemRead,PCWrite,IFIDWrite,HazMuxCon); 
	Forwarding Unit
		-ForwardUnit(MEMRegRd,WBRegRd,EXRegRs,EXRegRt, MEM_RegWrite, WB_RegWrite, ForwardA, ForwardB); 
5- Hnaa Adel
	Memory / WriteBack Register 
		-module MEMWB(clock,WB,Memout,ALUOut,RegRD,WBreg,Memreg,ALUreg,RegRDreg); 

*/









/*
---------------------------------------------------------------------------------------------------------------------------------
IF/ID Pipeline Register 
Verina Alber
---------------------------------------------------------------------------------------------------------------------------------

*/
//
//`timescale 1 ps / 100 fs
`include "Single_Cycle_Mips.v"
module IFID(flush,clock,IFIDWrite,PC_Plus4,Inst,InstReg,PC_Plus4Reg); 
    input [31:0] PC_Plus4,Inst; 
    input clock,IFIDWrite,flush; 
    output [31:0] InstReg, PC_Plus4Reg; 
     
    reg [31:0] InstReg, PC_Plus4Reg; 
     
    initial begin 
        InstReg = 0; 
        PC_Plus4Reg = 0; 
    end 
     
    always@(posedge clock) 
    begin 
        if(flush) 
        begin 
           InstReg <= 0; 
           PC_Plus4Reg <=0; 
        end 
        else if(IFIDWrite) 
        begin 
           InstReg <= Inst; 
           PC_Plus4Reg <= PC_Plus4; 
        end 
    end 
 
endmodule  


/*
---------------------------------------------------------------------------------------------------------------------------------
ID/Ex Pipeline Register 
Marina Saad
---------------------------------------------------------------------------------------------------------------------------------

*/



module IDEX(clock,WB,M,EX,DataA,DataB,imm_value,RegRs,RegRt,RegRd,WBreg,Mreg,EXreg,DataAreg, 
DataBreg,imm_valuereg,RegRsreg,RegRtreg,RegRdreg); 
    input clock; 
    input [1:0] WB; 
    input [2:0] M; 
    input [3:0] EX; 
    input [4:0] RegRs,RegRt,RegRd; 
    input [31:0] DataA,DataB,imm_value; 
    output [1:0] WBreg; 
    output [2:0] Mreg; 
    output [3:0] EXreg; 
    output [4:0] RegRsreg,RegRtreg,RegRdreg; 
    output [31:0] DataAreg,DataBreg,imm_valuereg; 
     
 
    reg [1:0] WBreg; 
    reg [2:0] Mreg; 
    reg [3:0] EXreg; 
    reg [31:0] DataAreg,DataBreg,imm_valuereg; 
    reg [4:0] RegRsreg,RegRtreg,RegRdreg; 
     
    initial begin 
        WBreg = 0; 
        Mreg = 0; 
        EXreg = 0; 
        DataAreg = 0; 
        DataBreg = 0; 
        imm_valuereg = 0; 
        RegRsreg = 0; 
        RegRtreg = 0; 
        RegRdreg = 0; 
    end 
     
    always@(posedge clock) 
    begin 
        WBreg <= WB; 
        Mreg <= M; 
        EXreg <= EX; 
        DataAreg <= DataA; 
        DataBreg <= DataB; 
        imm_valuereg <= imm_value; 
        RegRsreg <= RegRs; 
        RegRtreg <= RegRt; 
        RegRdreg <= RegRd; 
    end 
     
endmodule 


/*
---------------------------------------------------------------------------------------------------------------------------------
Ex/MEM Pipeline Register 
Mina Magdy
---------------------------------------------------------------------------------------------------------------------------------

*/

module EXMEM(clock,WB,M,ALUOut,RegRD,WriteDataIn,Mreg,WBreg,ALUreg,RegRDreg,WriteDataOut); 
   input clock; 
   input [1:0] WB; 
   input [2:0] M; 
   input [4:0] RegRD; 
   input [31:0] ALUOut,WriteDataIn; 
   output [1:0] WBreg; 
   output [2:0] Mreg; 
   output [31:0] ALUreg,WriteDataOut; 
   output [4:0] RegRDreg; 
 
   reg [1:0] WBreg; 
   reg [2:0] Mreg; 
   reg [31:0] ALUreg,WriteDataOut; 
   reg [4:0] RegRDreg; 
    
   initial begin 
      WBreg=0; 
      Mreg=0; 
      ALUreg=0; 
      WriteDataOut=0; 
      RegRDreg=0; 
   end 
    
    
    always@(posedge clock) 
    begin 
        WBreg <= WB; 
        Mreg <= M; 
        ALUreg <= ALUOut; 
        RegRDreg <= RegRD; 
        WriteDataOut <= WriteDataIn; 
    end 
 
endmodule 




/*
---------------------------------------------------------------------------------------------------------------------------------
MEM/WB Pipeline Register 
Hnaa Adel
---------------------------------------------------------------------------------------------------------------------------------

*/
module MEMWB(clock,WB,Memout,ALUOut,RegRD,WBreg,Memreg,ALUreg,RegRDreg); 
   input clock; 
   input [1:0] WB; 
   input [4:0] RegRD; 
   input [31:0] Memout,ALUOut; 
   output [1:0] WBreg; 
   output [31:0] Memreg,ALUreg; 
   output [4:0] RegRDreg; 
 
   reg [1:0] WBreg; 
   reg [31:0] Memreg,ALUreg; 
   reg [4:0] RegRDreg; 
    
   initial begin 
      WBreg = 0; 
      Memreg = 0; 
      ALUreg = 0; 
      RegRDreg = 0; 
        
   end 
    
    always@(posedge clock) 
    begin 
        WBreg <= WB; 
        Memreg <= Memout; 
        ALUreg <= ALUOut; 
        RegRDreg <= RegRD; 
    end 
    
endmodule 

/*
---------------------------------------------------------------------------------------------------------------------------------
Forward Unit 
Mina Mourice
---------------------------------------------------------------------------------------------------------------------------------

*/



module ForwardUnit(MEMRegRd,WBRegRd,EXRegRs,EXRegRt, MEM_RegWrite, WB_RegWrite, ForwardA, ForwardB); 
   input[4:0] MEMRegRd,WBRegRd,EXRegRs,EXRegRt;  
   input MEM_RegWrite, WB_RegWrite; 
   output[1:0] ForwardA, ForwardB; 
 
   reg[1:0] ForwardA, ForwardB; 
    
   /*Forward A*/ 
   always@(MEM_RegWrite or MEMRegRd or EXRegRs or WB_RegWrite or WBRegRd) 
   begin 
      if((MEM_RegWrite)&&(MEMRegRd != 0)&&(MEMRegRd == EXRegRs)) 
         ForwardA = 2'b10; 
      else if((WB_RegWrite)&&(WBRegRd != 0)&&(WBRegRd == EXRegRs)&&(MEMRegRd != EXRegRs) ) 
         ForwardA = 2'b01; 
      else 
         ForwardA = 2'b00; 
   end 
 
   /*Forward B */
   always@(WB_RegWrite or WBRegRd or EXRegRt or MEMRegRd or MEM_RegWrite) 
   begin 
      if((WB_RegWrite)&&(WBRegRd != 0)&&(WBRegRd == EXRegRt)&&(MEMRegRd != EXRegRt) ) 
         ForwardB = 2'b01; 
      else if((MEM_RegWrite)&&(MEMRegRd != 0)&&(MEMRegRd == EXRegRt)) 
         ForwardB = 2'b10; 
      else  
         ForwardB = 2'b00; 
   end 
 
endmodule


/*
---------------------------------------------------------------------------------------------------------------------------------
Hazard Unit 
Mina Mourice
---------------------------------------------------------------------------------------------------------------------------------

*/


module HazardUnit(IDRegRs,IDRegRt,EXRegRt,EXMemRead,PCWrite,IFIDWrite,HazMuxCon); 
    input [4:0] IDRegRs,IDRegRt,EXRegRt; 
    input EXMemRead; 
    output PCWrite, IFIDWrite, HazMuxCon; 
     
    reg PCWrite, IFIDWrite, HazMuxCon; 
     
    always@(IDRegRs,IDRegRt,EXRegRt,EXMemRead) 
    if(EXMemRead&((EXRegRt == IDRegRs)|(EXRegRt == IDRegRt))) 
       begin//stall 
           PCWrite = 0; 
           IFIDWrite = 0; 
           HazMuxCon = 1; 
       end 
    else 
       begin//no stall 
           PCWrite = 1; 
           IFIDWrite = 1; 
           HazMuxCon = 1; 
     
       end 
 
endmodule 








/*
---------------------------------------------------------------------------------------------------------------------------------
Main CPU it Contains :
1- input clock
2- Call for all stages (Instruction_Fetch,Instruction_Decode,Instruction_Execute,Instruction_Memory,Write_Back)
3-  
---------------------------------------------------------------------------------------------------------------------------------

*/
module cpu(clock); 
    input clock; 
     
    //debugging vars 
    reg [31:0] cycle; 
     
    //IF vars 
    wire [31:0] nextpc,IFpc_plus_4,IFinst; 
    reg [31:0] pc; 
     
    //ID vars 
    wire PCSrc; 
    wire [4:0] IDRegRs,IDRegRt,IDRegRd; 
    wire [31:0] IDpc_plus_4,IDinst; 
    wire [31:0] IDRegAout, IDRegBout; 
    wire [31:0] IDimm_value,BranchAddr,PCMuxOut,JumpTarget; 
     
    //control vars in ID stage 
    wire PCWrite,IFIDWrite,HazMuxCon,jump,bne,imm,andi,ori,addi; 
    wire [8:0] IDcontrol,ConOut; 
         
    //EX vars 
    wire [1:0] EXWB,ForwardA,ForwardB,aluop; 
    wire [2:0] EXM; 
    wire [3:0] EXEX,ALUCon; 
    wire [4:0] EXRegRs,EXRegRt,EXRegRd,regtopass; 
    wire [31:0] EXRegAout,EXRegBout,EXimm_value, b_value; 
    wire [31:0] EXALUOut,ALUSrcA,ALUSrcB; 
     
    //MEM vars 
    wire [1:0] MEMWB; 
    wire [2:0] MEMM; 
    wire [4:0] MEMRegRd; 
    wire [31:0] MEMALUOut,MEMWriteData,MEMReadData; 
     
    //WB vars 
    wire [1:0] WBWB; 
    wire [4:0] WBRegRd; 
    wire [31:0] datatowrite,WBReadData,WBALUOut; 
     
    
    //initial conditions 
    initial begin 
       pc = 0; 
       cycle = 0; 
    end 
     
    //debugging variable 
    always@(posedge clock) 
    begin 
       cycle = cycle + 1; 
    end 
     
    /** 
     * Instruction Fetch (IF) 
     */ 
    assign PCSrc = ((IDRegAout==IDRegBout)&IDcontrol[6])|((IDRegAout!=IDRegBout)&bne); 
    assign IFFlush = PCSrc|jump; 
    assign IFpc_plus_4 = pc + 4; 
 
    assign nextpc = PCSrc ? BranchAddr : PCMuxOut; 
     
  
 
    always @ (posedge clock) begin 
       if(PCWrite) 
       begin 
          pc = nextpc; //update pc 
          $display("PC: %d",pc); 
       end 
       else 
          $display("Skipped writting to PC - nop"); //nop dont update 
    end     
     
    InstructMem IM(pc,IFinst); 
     
    IFID IFIDreg(IFFlush,clock,IFIDWrite,IFpc_plus_4,IFinst,IDinst,IDpc_plus_4); 
    /** 
     * Instruction Decode (ID) 
     */ 
     assign IDRegRs[4:0]=IDinst[25:21]; 
   assign IDRegRt[4:0]=IDinst[20:16]; 
   assign IDRegRd[4:0]=IDinst[15:11]; 
   assign IDimm_value = {IDinst[15],IDinst[15],IDinst[15],IDinst[15],IDinst[15],IDinst[15],IDinst[15],IDinst[15],IDinst[15],IDinst[15],IDinst[15],IDinst[15],IDinst[15],
IDinst[15],IDinst[15],IDinst[15],IDinst[15:0]}; 
   assign BranchAddr = (IDimm_value << 2) + IDpc_plus_4; 
   assign JumpTarget[31:28] = IFpc_plus_4[31:28]; 
   assign JumpTarget[27:2] = IDinst[25:0]; 
   assign JumpTarget[1:0] = 0; 
    
   assign IDcontrol = HazMuxCon ? ConOut : 0;  
    assign PCMuxOut = jump ? JumpTarget : IFpc_plus_4; 
    
   HazardUnit HU(IDRegRs,IDRegRt,EXRegRt,EXM[1],PCWrite,IFIDWrite,HazMuxCon); 
   Control thecontrol(IDinst[31:26],ConOut,jump,bne,imm,andi,ori,addi); 
    Registers piperegs(clock,WBWB[0],datatowrite,WBRegRd,IDRegRs,IDRegRt,IDRegAout,IDRegBout); 
     
    IDEX IDEXreg(clock,IDcontrol[8:7],IDcontrol[6:4],IDcontrol[3:0],IDRegAout,IDRegBout,IDimm_value
,IDRegRs,IDRegRt,IDRegRd,EXWB,EXM,EXEX,EXRegAout,EXRegBout,EXimm_value,EXRegRs,EXRegRt,EXRegRd);                                                         
    /** 
     * Execution (EX) 
     */ 
    assign regtopass = EXEX[3] ? EXRegRd : EXRegRt; 
    assign b_value = EXEX[2] ? EXimm_value : EXRegBout; 
     
    BIGMUX2 MUX0(ForwardA,EXRegAout,datatowrite,MEMALUOut,0,ALUSrcA); 
    BIGMUX2 MUX1(ForwardB,b_value,datatowrite,MEMALUOut,0,ALUSrcB); 
    ForwardUnit FU(MEMRegRd,WBRegRd,EXRegRs, EXRegRt, MEMWB[0], WBWB[0], ForwardA, ForwardB); 
    // ALU control 
   assign aluop[0] = 
(~IDinst[31]&~IDinst[30]&~IDinst[29]&IDinst[28]&~IDinst[27]&~IDinst[26])|(imm); 
   assign aluop[1] = 
(~IDinst[31]&~IDinst[30]&~IDinst[29]&~IDinst[28]&~IDinst[27]&~IDinst[26])|(imm); 
    ALUControl ALUcontrol(andi,ori,addi,EXEX[1:0],EXimm_value[5:0],ALUCon); 
    ALU theALU(ALUCon,ALUSrcA,ALUSrcB,EXALUOut); 
     
    EXMEM EXMEMreg(clock,EXWB,EXM,EXALUOut,regtopass,EXRegBout,MEMM,MEMWB,MEMALUOut,MEMRegRd,MEMWriteData); 
    /** 
     * Memory (Mem) 
     */ 
    DATAMEM DM(MEMM[0],MEMM[1],MEMALUOut,MEMWriteData,MEMReadData); 
     

    MEMWB MEMWBreg(clock,MEMWB,MEMReadData,MEMALUOut,MEMRegRd,WBWB,WBReadData,WBALUOut,WBRegRd); 
    /** 
     * Write Back (WB) 
     */ 
    assign datatowrite = WBWB[1] ? WBReadData : WBALUOut; 
     
     
endmodule 
module Pipelined_TestBench; 
     
   reg Clock; 
   integer i; 
 
   initial begin 
       Clock = 1; 
   end 
   //clock controls 
   always begin 
        Clock = ~Clock; 
        #25; 
  end 
   
  initial begin 
       

 
  // Data Memory intialization 
  pipelined.DM.regfile[0] = 32'd8; 
  pipelined.DM.regfile[1] = 32'd1; 
  pipelined.DM.regfile[2] = -32'd1; 
  pipelined.DM.regfile[3] = 0; 
   
  pipelined.piperegs.regfile[0] = 0; 

  // Register File initialization 
  for (i = 0; i < 32; i = i + 1) 
    pipelined.piperegs.regfile[i] = 32'd0; 
   end 
  //Instantiate cpu 
   cpu pipelined(Clock); 
     
endmodule 
module Pipelined_TestBenchA; 
     
   reg Clock; 
   integer i; 
 
   initial begin 
       Clock = 1; 
   end 
   //clock controls 
   always begin 
        Clock = ~Clock; 
        #25; 
  end 
   
   initial begin 
       /*
  // Instr Memory intialization 
  pipelined.IM.regfile[0] = 32'h8c030000; 
  pipelined.IM.regfile[4] = 32'h8c040001; 
  pipelined.IM.regfile[8] = 32'h8c050002; 
  pipelined.IM.regfile[12] = 32'h8c010002; 
  pipelined.IM.regfile[16] = 32'h10600004; 
  pipelined.IM.regfile[20] = 32'h00852020; 
  pipelined.IM.regfile[24] = 32'h00852822; 
  pipelined.IM.regfile[28] = 32'h00611820; 
  pipelined.IM.regfile[32] = 32'h1000fffb; 
  pipelined.IM.regfile[36] = 32'hac040006; 
 */
  // Data Memory intialization 
  pipelined.DM.regfile[0] = 32'd9; 
  pipelined.DM.regfile[1] = 32'd1; 
  pipelined.DM.regfile[2] = -32'd1; 
  pipelined.DM.regfile[3] = 0; 
   
  pipelined.piperegs.regfile[0] = 0; 
 
  // Register File initialization 
  for (i = 0; i < 32; i = i + 1) 
    pipelined.piperegs.regfile[i] = 32'd0; 
 
 
  end 
  //Instantiate cpu 
   cpu pipelined(Clock); 
     
endmodule 
 
 
module Pipelined_TestBenchB; 
     
   reg Clock; 
   integer i; 
 
   initial begin 
       Clock = 1; 
   end 
   //clock controls 
   always begin 
        Clock = ~Clock; 
        #25; 
  end 
   
   initial begin 
       
  // Instr Memory intialization 
  pipelined.piperegs.regfile[0] = 32'b00111000000100000000000000000011; //lw R3,0(R1) 
  pipelined.piperegs.regfile[4] = 32'h8C040001;//lw R4,1(R0) 
  pipelined.piperegs.regfile[8] = 32'h00642820;//add R5,R3,R4 
  pipelined.piperegs.regfile[12] = 32'h00A43022;//sub R6,R5,R4 
  pipelined.piperegs.regfile[16] = 32'h00643824;//and R7,R3,R4 
  pipelined.piperegs.regfile[20] = 32'h00644025;//or R8,R3,R4 
  pipelined.piperegs.regfile[24] = 32'h00644827;//nor R9,R3,R4 
  pipelined.IM.regfile[28] = 32'h00C5502A;//slt R10,R6,R5 
  pipelined.IM.regfile[32] = 32'h80000008;//j startloop 
  pipelined.IM.regfile[36] = 32'h2063FFFF;//loop: addi R3,R3,-1 
  pipelined.IM.regfile[40] = 32'h14E3FFFE;//startloop: bne R3,R7,-2 
  pipelined.IM.regfile[44] = 32'h01295818;//mult R11,R9,R9 
  pipelined.IM.regfile[48] = 32'h0166601A;//div R12,R11,R6 
  pipelined.IM.regfile[52] = 32'h34CE0002;//ori R14,R6,2 
  pipelined.IM.regfile[56] = 32'h11CC0000;//beq R14,R12, next 
  pipelined.IM.regfile[60] = 32'hADCE0006;//sw  
   
  // Data Memory intialization 
  pipelined.DM.regfile[0] = 32'd8; 
  pipelined.DM.regfile[1] = 32'd1; 
   
  pipelined.piperegs.regfile[0] = 0; 
 
  // Register File initialization 
  for (i = 0; i < 32; i = i + 1) 
    pipelined.piperegs.regfile[i] = 32'd0; 
 
 
  end 
  //Instantiate cpu 
   cpu pipelined(Clock); 
     
endmodule 