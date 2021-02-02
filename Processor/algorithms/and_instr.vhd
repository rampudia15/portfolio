LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

LIBRARY work;
USE work.processor_types.ALL;

PACKAGE and_instr IS

   FUNCTION and_behav (rs:IN std_logic_vector; rt:IN std_logic_vector) RETURN std_logic_vector;
   FUNCTION and_algo1 (rs:IN std_logic_vector; rt:IN std_logic_vector) RETURN std_logic_vector;
   FUNCTION and_algo2 (rs:IN std_logic_vector; rt:IN std_logic_vector) RETURN std_logic_vector;

END and_instr;


PACKAGE BODY and_instr IS

   FUNCTION and_behav (rs:IN std_logic_vector; rt:IN std_logic_vector) RETURN std_logic_vector IS
   BEGIN
      RETURN rs AND rt;
   END and_behav;

   FUNCTION and_algo1 (rs:IN std_logic_vector; rt:IN std_logic_vector) RETURN std_logic_vector IS
      VARIABLE rd : bit32 := (OTHERS => '0');
   BEGIN
      FOR i in rd'RANGE LOOP
         IF rs(i) = '1' AND rt(i) = '1' THEN
            rd(i) := '1';
         END IF;
      END LOOP;
      RETURN rd;
   END and_algo1;

   FUNCTION and_algo2 (rs:IN std_logic_vector; rt:IN std_logic_vector) RETURN std_logic_vector IS
      VARIABLE rd : bit32 := rs;
   BEGIN
      FOR i in rd'RANGE LOOP
         IF rt(i) = '0' THEN
            rd(i) := '0';
         END IF;
      END LOOP;
      RETURN rd;
   END and_algo2;

END PACKAGE BODY and_instr;