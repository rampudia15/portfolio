LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

LIBRARY work;
USE work.processor_types.ALL;

PACKAGE or_instr IS

   FUNCTION or_behav (rs:IN std_logic_vector; rt:IN std_logic_vector) RETURN std_logic_vector;
   FUNCTION or_algo1 (rs:IN std_logic_vector; rt:IN std_logic_vector) RETURN std_logic_vector;
   FUNCTION or_algo2 (rs:IN std_logic_vector; rt:IN std_logic_vector) RETURN std_logic_vector;

END or_instr;


PACKAGE BODY or_instr IS

   FUNCTION or_behav (rs:IN std_logic_vector; rt:IN std_logic_vector) RETURN std_logic_vector IS
   BEGIN
      RETURN rs OR rt;
   END or_behav;

   FUNCTION or_algo1 (rs:IN std_logic_vector; rt:IN std_logic_vector) RETURN std_logic_vector IS
      VARIABLE rd : bit32 := (OTHERS => '0');
   BEGIN
      FOR i in rd'RANGE LOOP
         IF rs(i) = '1' OR rt(i) = '1' THEN
            rd(i) := '1';
         END IF;
      END LOOP;
      RETURN rd;
   END or_algo1;

   FUNCTION or_algo2 (rs:IN std_logic_vector; rt:IN std_logic_vector) RETURN std_logic_vector IS
      VARIABLE rd : bit32 := rs;
   BEGIN
      FOR i in rd'RANGE LOOP
         IF rt(i) = '1' THEN
            rd(i) := '1';
         END IF;
      END LOOP;
      RETURN rd;
   END or_algo2;

END PACKAGE BODY or_instr;
