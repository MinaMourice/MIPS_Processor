MIPS_Processor
===============================================

Implementation of MIPS Processor using verilog description language
--------------------------------------------------------------------


Implementing:

- MIPS Processor
- Hazards

Single Process Instruction:
---------------------------
Implementing:

- module Program_Counter(PC_in,PC_out)  
- module Instruction_Memory (Read_address,Instruction) 
- module Adder(in_1,in_2,out) 
- module control_unit(Inst31_26,Reg_Dst,Branch,Mem_Read,Mem_To_Reg,ALU_Op,Mem_Write,ALU_Src,Reg_Write)
- module shift_left2(in,out)
- module sign_extend(in,out)
- module Data_Memory(Address,Write_Data,Read_Data)
- module Or_gate(in_1,in_2,out)
