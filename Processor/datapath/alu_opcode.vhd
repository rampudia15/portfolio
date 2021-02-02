LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

LIBRARY work;
USE work.processor_types.ALL;

PACKAGE alu_opcode IS

   CONSTANT alu_and:	bit4 := "0001";
   CONSTANT alu_or:	bit4 := "0010";
   CONSTANT alu_add:	bit4 := "0011";
   CONSTANT alu_sub:	bit4 := "0100";
   CONSTANT alu_div:	bit4 := "0101";
   CONSTANT alu_mult:	bit4 := "0110";
   CONSTANT alu_mflo:	bit4 := "0111";
   CONSTANT alu_mfhi:	bit4 := "1000";
   CONSTANT alu_slt:	bit4 := "1001";
   CONSTANT alu_lui:	bit4 := "1010";
   CONSTANT alu_beq:	bit4 := "1011";
   CONSTANT alu_bgez:	bit4 := "1100";
   CONSTANT alu_ori:	bit4 := "1101";
   CONSTANT alu_nop:	bit4 := "1111";
  
END alu_opcode;
