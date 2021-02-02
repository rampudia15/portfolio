LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

LIBRARY work;
USE work.processor_types.ALL;

PACKAGE ori_instr IS

   FUNCTION ori_behav (rs:IN std_logic_vector; imm:IN std_logic_vector) RETURN std_logic_vector;
   FUNCTION ori_algo1 (rs:IN std_logic_vector; imm:IN std_logic_vector) RETURN std_logic_vector;
   FUNCTION ori_algo2 (rs:IN std_logic_vector; imm:IN std_logic_vector) RETURN std_logic_vector;

END ori_instr;


PACKAGE BODY ori_instr IS

   FUNCTION ori_behav (rs:IN std_logic_vector; imm:IN std_logic_vector) RETURN std_logic_vector IS
   BEGIN
      RETURN rs OR (X"0000" & imm);
   END ori_behav;

   FUNCTION ori_algo1 (rs:IN std_logic_vector; imm:IN std_logic_vector) RETURN std_logic_vector IS
      VARIABLE rt : bit32 := rs;
   BEGIN
      FOR i in imm'RANGE LOOP
         IF rs(i) = '1' OR imm(i) = '1' THEN
            rt(i) := '1';
         END IF;
      END LOOP;
      RETURN rt;
   END ori_algo1;

   FUNCTION ori_algo2 (rs:IN std_logic_vector; imm:IN std_logic_vector) RETURN std_logic_vector IS
      VARIABLE rt : bit32 := rs;
   BEGIN
      FOR i in imm'RANGE LOOP
         IF imm(i) = '1' THEN
            rt(i) := '1';
         END IF;
      END LOOP;
      RETURN rt;
   END ori_algo2;

END PACKAGE BODY ori_instr;
