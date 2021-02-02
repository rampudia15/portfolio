LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

LIBRARY work;
USE work.processor_types.ALL;

PACKAGE slt_instr IS

   FUNCTION slt (rs:IN std_logic_vector; rt:IN std_logic_vector) RETURN std_logic_vector;
 
END slt_instr;


PACKAGE BODY slt_instr IS

   FUNCTION slt (rs:IN std_logic_vector; rt:IN std_logic_vector) RETURN std_logic_vector IS
      VARIABLE comparator_slt : std_logic_vector(31 DOWNTO 0) := X"00000000";
      VARIABLE output : bit32 := (OTHERS => '0');			

   BEGIN
      comparator_slt := std_logic_vector(signed(rt) + signed(not(rs)));
      CASE rt(31) xnor rs(31) IS 
         WHEN '0' => -- Rt and Rs have different sign
            IF rt(31) = '0' THEN -- Rt is positive, Rs is negative
               output := (0 => '1', OTHERS => '0');
            ELSIF rt(31) = '1' THEN -- Rt is negative, Rs is positive
               output := (OTHERS => '0');
            END IF;
         WHEN '1' => -- Rt and Rs have same sign
            IF comparator_slt(31) = '0' THEN -- Rt - Rs is positive
               output := (0 => '1', OTHERS => '0');
            ELSIF comparator_slt(31) = '1' THEN -- Rt - Rs is negative
               output := (OTHERS => '0');
            END IF;
         WHEN OTHERS => -- Do nothing
      END CASE;
      RETURN output;
   END slt;

END PACKAGE BODY slt_instr;