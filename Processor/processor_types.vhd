LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

PACKAGE processor_types IS
   SUBTYPE bit64 IS std_logic_vector (63 DOWNTO 0);
   SUBTYPE bit32 IS std_logic_vector (31 DOWNTO 0);
   SUBTYPE bit16 IS std_logic_vector (15 DOWNTO 0);
   SUBTYPE bit8  IS std_logic_vector  (7 DOWNTO 0);
   SUBTYPE bit6  IS std_logic_vector  (5 DOWNTO 0);
   SUBTYPE bit5  IS std_logic_vector  (4 DOWNTO 0);
   SUBTYPE bit4  IS std_logic_vector  (3 DOWNTO 0);
   SUBTYPE bit3  IS std_logic_vector  (2 DOWNTO 0);
   SUBTYPE bit2  IS std_logic_vector  (1 DOWNTO 0);

   TYPE registers IS ARRAY (0 TO 31) OF bit32;

   -- instruction set

   -- R-Type instructions function codes
   CONSTANT op_rtype:	bit6 := "000000";
   CONSTANT func_and:	bit6 := "100100";
   CONSTANT func_or:	bit6 := "100101";
   CONSTANT func_add:	bit6 := "100000";
   CONSTANT func_sub:	bit6 := "100010";
   CONSTANT func_div:	bit6 := "011010";
   CONSTANT func_mflo:	bit6 := "010010";
   CONSTANT func_mfhi:	bit6 := "010000";
   CONSTANT func_mult:	bit6 := "011000";
   CONSTANT func_slt:	bit6 := "101010";
   CONSTANT func_nop:	bit6 := "000000";

   -- I-Type instructions op codes
   CONSTANT op_bgez:	bit6 := "000001";
   CONSTANT op_beq:	bit6 := "000100";
   CONSTANT op_ori:	bit6 := "001101";
   CONSTANT op_addi:	bit6 := "001000";
   CONSTANT op_lui:	bit6 := "001111";
   CONSTANT op_lw:	bit6 := "100011";
   CONSTANT op_sw:	bit6 := "101011";

   -- Instruction NOP
   CONSTANT nop:	bit32 := (OTHERS => '0');

   CONSTANT dontcare:	bit32 := (OTHERS => '-');
   CONSTANT undefined:	bit32 := (OTHERS => 'X');
  
END processor_types;