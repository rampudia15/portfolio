LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

LIBRARY work;
USE work.processor_types.ALL;

PACKAGE sub_instr IS

   FUNCTION sub_behav (rs:IN std_logic_vector; rt:IN std_logic_vector) RETURN std_logic_vector;
   FUNCTION sub_algo1 (rs:IN std_logic_vector; rt:IN std_logic_vector) RETURN std_logic_vector;
   FUNCTION sub_algo2 (rs:IN std_logic_vector; rt:IN std_logic_vector) RETURN std_logic_vector;

END sub_instr;


PACKAGE BODY sub_instr IS

   FUNCTION sub_behav (rs:IN std_logic_vector; rt:IN std_logic_vector) RETURN std_logic_vector IS
   BEGIN
      RETURN std_logic_vector(signed(rs) - signed(rt));
   END sub_behav;

   -- x - y = x + (-y)
   FUNCTION sub_algo1 (rs:IN std_logic_vector; rt:IN std_logic_vector) RETURN std_logic_vector IS
      VARIABLE rt_2s_complement : bit32 := rt;
   BEGIN
      rt_2s_complement := std_logic_vector(1 + signed(not(rt))); -- => 2's complement of rt
      RETURN std_logic_vector(signed(rs) + signed(rt_2s_complement));
   END sub_algo1;

   -- Binary substraction
   FUNCTION sub_algo2 (rs:IN std_logic_vector; rt:IN std_logic_vector) RETURN std_logic_vector IS
      VARIABLE rd : bit32 := (OTHERS => '0');
      VARIABLE borrow : std_logic := '0';
   BEGIN
      FOR i in 0 TO 31 LOOP
         IF borrow = '0' THEN
            IF rs(i) = rt(i) THEN
               rd(i) := '0';
            ELSIF rs(i) = '1' AND rt(i) = '0' THEN
               rd(i) := '1';
            ELSE
               rd(i) := '1';
               borrow := '1';
            END IF;
         ELSE
            IF rs(i) = rt(i) THEN
               rd(i) := '1';
               borrow := '1';
            ELSIF rs(i) = '1' AND rt(i) = '0' THEN
               rd(i) := '0';
               borrow := '0';
            ELSE
               rd(i) := '0';
               borrow := '1';
            END IF;
         END IF;
      END LOOP;
      RETURN rd;
   END sub_algo2;

END PACKAGE BODY sub_instr;