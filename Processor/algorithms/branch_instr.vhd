LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

LIBRARY work;
USE work.processor_types.ALL;

PACKAGE branch_instr IS

   FUNCTION beq_algo (rs:IN std_logic_vector; rt:IN std_logic_vector) RETURN std_logic_vector;
   FUNCTION bgez_algo (rs:IN std_logic_vector) RETURN std_logic_vector;
 
END branch_instr;


PACKAGE BODY branch_instr IS

   FUNCTION beq_algo (rs:IN std_logic_vector; rt:IN std_logic_vector) RETURN std_logic_vector IS
      VARIABLE output : bit32 := (OTHERS => '0'); -- Only bit 0 is relevant

   BEGIN
      output(0) := (rs(31) XNOR rt(31)) AND
                   (rs(30) XNOR rt(30)) AND
                   (rs(29) XNOR rt(29)) AND
                   (rs(28) XNOR rt(28)) AND
                   (rs(27) XNOR rt(27)) AND
                   (rs(26) XNOR rt(26)) AND
                   (rs(25) XNOR rt(25)) AND
                   (rs(24) XNOR rt(24)) AND
                   (rs(23) XNOR rt(23)) AND
                   (rs(22) XNOR rt(22)) AND
                   (rs(21) XNOR rt(21)) AND
                   (rs(20) XNOR rt(20)) AND
                   (rs(19) XNOR rt(19)) AND
                   (rs(18) XNOR rt(18)) AND
                   (rs(17) XNOR rt(17)) AND
                   (rs(16) XNOR rt(16)) AND
                   (rs(15) XNOR rt(15)) AND
                   (rs(14) XNOR rt(14)) AND
                   (rs(13) XNOR rt(13)) AND
                   (rs(12) XNOR rt(12)) AND
                   (rs(11) XNOR rt(11)) AND
                   (rs(10) XNOR rt(10)) AND
                   (rs(9) XNOR rt(9)) AND
                   (rs(8) XNOR rt(8)) AND
                   (rs(7) XNOR rt(7)) AND
                   (rs(6) XNOR rt(6)) AND
                   (rs(5) XNOR rt(5)) AND
                   (rs(4) XNOR rt(4)) AND
                   (rs(3) XNOR rt(3)) AND
                   (rs(2) XNOR rt(2)) AND
                   (rs(1) XNOR rt(1)) AND
                   (rs(0) XNOR rt(0));      
      RETURN output;
   END beq_algo;

   FUNCTION bgez_algo (rs:IN std_logic_vector) RETURN std_logic_vector IS
	VARIABLE output : bit32 := (OTHERS => '0'); -- Only bit 0 is relevant
   BEGIN
      IF rs(31) = '0' THEN
         output(0) := '1';
      END IF;
      RETURN output;
   END bgez_algo;

END PACKAGE BODY branch_instr;
