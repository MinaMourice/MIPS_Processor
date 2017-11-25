MIPS_Processor
===============================================

Implementation of MIPS Processor using verilog description language
--------------------------------------------------------------------


Implementing:

- MIPS Processor
- Hazards

Single Process Instruction:
---------------------------
::
- module Program_Counter(PC_in,PC_out)  //Verina Alber
- module Instruction_Memory (Read_address,Instruction) //Marina Saad
- module Adder(in_1,in_2,out) //Hnaa Adel
- module control_unit(Inst31_26,Reg_Dst,Branch,Mem_Read,Mem_To_Reg,ALU_Op,Mem_Write,ALU_Src,Reg_Write)//Mina Mourice
- module shift_left2(in,out)//Mina Magdy
- module sign_extend(in,out)//Mina Magdy
- module Data_Memory(Address,Write_Data,Read_Data)//Mina Mourice
- module Or_gate(in_1,in_2,out)//Hnaa Adel
