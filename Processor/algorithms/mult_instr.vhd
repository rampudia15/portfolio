LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

PACKAGE mult_instr IS

   FUNCTION signed_multiply(op1,op2 : signed) RETURN signed; -- addshift algorithm
   FUNCTION addshift_multiply(op1u,op2u : unsigned) RETURN unsigned; -- addshift for unsigned multiplication
   FUNCTION booth_multiply(op1,op2 : signed) RETURN signed; -- booth algorithm with 65 bits adder
   FUNCTION booth_multiply2(op1,op2 : signed) RETURN signed; -- booth algorithm with 32 bits adder
   FUNCTION convert(inp : std_logic_vector) RETURN std_logic_vector;
   
   FUNCTION signed_multiply_separate (op1,op2 : signed) RETURN signed;
   FUNCTION multiply_separate(op1,op2 : unsigned) RETURN unsigned; -- separate hi and lo with 16 bits adder

END mult_instr;


PACKAGE BODY mult_instr IS

   FUNCTION signed_multiply (op1,op2 : signed) RETURN signed IS
      VARIABLE op1u : unsigned(op1'length-1 DOWNTO 0); -- multiplier
      VARIABLE op2u : unsigned(op2'length-1 DOWNTO 0); -- multiplicant
      VARIABLE res  : unsigned(op1'length+op2'length-1 DOWNTO 0);
   BEGIN
      -- case of the signs of both factors
      -- same: result positive
      IF op1(op1'length-1) = '0' AND op2(op2'length-1) = '0' THEN
         op1u := unsigned(op1);
         op2u := unsigned(op2);
         res := addshift_multiply(op1u, op2u);
         RETURN signed(res);
      ELSIF op1(op1'length-1) = '1' AND op2(op2'length-1) = '1' THEN
         op1u := unsigned(convert(std_logic_vector(op1)));
         op2u := unsigned(convert(std_logic_vector(op2)));
         res := addshift_multiply(op1u, op2u);
         RETURN signed(res);
      -- different: result negative
      ELSIF op1(op1'length-1) = '0' AND op2(op2'length-1) = '1' THEN
         op1u := unsigned(op1);
         op2u := unsigned(convert(std_logic_vector(op2)));
         res := addshift_multiply(op1u, op2u);
         RETURN signed(convert(std_logic_vector(res)));
      ELSIF op1(op1'length-1) = '1' AND op2(op2'length-1) = '0' THEN
         op1u := unsigned(convert(std_logic_vector(op1)));
         op2u := unsigned(op2);
         res := addshift_multiply(op1u, op2u);
         RETURN signed(convert(std_logic_vector(res)));
      END IF;
   END signed_multiply;


   FUNCTION addshift_multiply (op1u,op2u : unsigned) RETURN unsigned IS
      VARIABLE tmp  : unsigned(op1u'length DOWNTO 0);
      VARIABLE op1i : unsigned(op1u'range);
   BEGIN
      op1i := op1u;
      tmp := (OTHERS => '0');
      FOR i IN 1 TO op1i'length LOOP
         IF op1i(0)='1' THEN
            tmp := tmp + op2u; --add
         END IF;
         op1i := tmp(0) & op1i(op1i'length-1 DOWNTO 1); -- shift
         tmp := '0' & tmp (op1i'length DOWNTO 1);
      END LOOP;
      RETURN tmp(op1i'length-1 DOWNTO 0) & op1i;
   END addshift_multiply;


   FUNCTION booth_multiply(op1,op2 : signed) RETURN signed IS -- op1 is multiplier,op2 is multiplicant
      VARIABLE A : signed(op1'length + op2'length DOWNTO 0);
      VARIABLE S : signed(op1'length + op2'length DOWNTO 0);
      VARIABLE P : signed(op1'length + op2'length DOWNTO 0);
      CONSTANT ZEROS_op1 : std_logic_vector(op2'length DOWNTO 0) := (OTHERS => '0');
      CONSTANT ZEROS_op2 : std_logic_vector(op1'length-1 DOWNTO 0) := (OTHERS => '0');
      CONSTANT MOST_NEG : signed(31 DOWNTO 0) := (31 => '1', OTHERS => '0');
   BEGIN
      A := signed(std_logic_vector(op2) & ZEROS_op1);
      S := signed(convert(std_logic_vector(op2)) & ZEROS_op1);
      P := signed(ZEROS_op2 & std_logic_vector(op1) & '0');
      FOR i IN 1 TO op1'length LOOP
         IF P(1) = '0' AND P(0) = '1' THEN
            P := P + A;
         ELSIF P(1) = '1' AND P(0) = '0' THEN
            P := P + S;
         ELSE
            P := P;
         END IF;
         P := P(op1'length+op2'length) & P(op1'length+op2'length DOWNTO 1); -- shift right
      END LOOP;
      IF op2 = MOST_NEG THEN
         RETURN signed(convert(std_logic_vector(P(op1'length+op2'length DOWNTO 1))));
      ELSE
         RETURN P(op1'length+op2'length DOWNTO 1);
      END IF;
   END booth_multiply;


   FUNCTION booth_multiply2 (op1,op2 : signed) RETURN signed IS
      VARIABLE tmp : signed(op2'length-1 DOWNTO 0); -- add/nothing multiplicant op2
      VARIABLE op1i : signed(op1'length-1 DOWNTO 0); -- store multiplier op1
      VARIABLE prev_bit : std_logic;
      CONSTANT MOST_NEG : signed(31 DOWNTO 0) := (31 => '1', OTHERS => '0');
   BEGIN
      op1i := op1;
      tmp := (OTHERS => '0');
      prev_bit := '0';
      FOR i IN 1 TO op1i'length LOOP
         IF op1i(0) = prev_bit THEN
            tmp := tmp; -- do nothing
         ELSIF op1i(0) = '1' AND prev_bit = '0' THEN
            tmp := tmp + signed(convert(std_logic_vector(op2))); -- substruct multiplicant
         ELSIF op1i(0) = '0' AND prev_bit = '1' THEN
            tmp := tmp + op2; -- add multiplicant
         END IF;
         prev_bit := op1i(0);
         op1i := tmp(0) & op1i(op1i'length-1 DOWNTO 1); -- shift right
         tmp  := tmp(op2'length-1) & tmp (op2'length-1 DOWNTO 1);
      END LOOP;
      IF op2 = MOST_NEG THEN
         RETURN signed(convert(std_logic_vector(tmp & op1i)));
      ELSE
         RETURN tmp & op1i;
      END IF;
   END booth_multiply2;


   FUNCTION convert(inp : std_logic_vector) RETURN std_logic_vector IS
      -- calculate the substruction
      VARIABLE outp : signed(inp'length-1 DOWNTO 0);
   BEGIN
      outp := signed(NOT(inp)) + 1;
      RETURN std_logic_vector(outp);
   END convert;


   FUNCTION signed_multiply_separate (op1,op2 : signed) RETURN signed IS
      VARIABLE op1u : unsigned(op1'length-1 DOWNTO 0); -- multiplier
      VARIABLE op2u : unsigned(op2'length-1 DOWNTO 0); -- multiplicant
      VARIABLE res : unsigned(op1'length + op2'length-1 DOWNTO 0);
   BEGIN
      -- case of the signs of both factors
      -- same: result positive
      IF op1(op1'length-1) = '0' AND op2(op2'length-1) = '0' THEN
         op1u := unsigned(op1);
         op2u := unsigned(op2);
         res := multiply_separate(op1u, op2u);
         RETURN signed(res);
      ELSIF op1(op1'length-1) = '1' AND op2(op2'length-1) = '1' THEN
         op1u := unsigned(convert(std_logic_vector(op1)));
         op2u := unsigned(convert(std_logic_vector(op2)));
         res := multiply_separate(op1u, op2u);
         RETURN signed(res);
      -- different: result negative
      ELSIF op1(op1'length-1) = '0' AND op2(op2'length-1) = '1' THEN
         op1u := unsigned(op1);
         op2u := unsigned(convert(std_logic_vector(op2)));
         res := multiply_separate(op1u, op2u);
         RETURN signed(convert(std_logic_vector(res)));
      ELSIF  op1(op1'length-1) = '1' AND op2(op2'length-1) = '0' THEN
         op1u := unsigned(convert(std_logic_vector(op1)));
         op2u := unsigned(op2);
         res := multiply_separate(op1u, op2u);
         RETURN signed(convert(std_logic_vector(res)));
      END IF;
   END signed_multiply_separate;


   FUNCTION multiply_separate(op1,op2 : unsigned) RETURN unsigned IS
      VARIABLE op1hi : unsigned(15 DOWNTO 0);
      VARIABLE op1lo : unsigned(15 DOWNTO 0);
      VARIABLE op2hi : unsigned(15 DOWNTO 0);
      VARIABLE op2lo : unsigned(15 DOWNTO 0);
      VARIABLE pl  : unsigned(31 DOWNTO 0);
      VARIABLE pm1 : unsigned(31 DOWNTO 0);
      VARIABLE pm2 : unsigned(31 DOWNTO 0);
      VARIABLE ph  : unsigned(31 DOWNTO 0);
      VARIABLE tmpm  : unsigned(32 DOWNTO 0);
      VARIABLE reslo : unsigned(32 DOWNTO 0);
      VARIABLE reshi : unsigned(31 DOWNTO 0);
      CONSTANT ZEROS : unsigned(14 DOWNTO 0) := (OTHERS => '0');
   BEGIN
      op1hi := op1(31 DOWNTO 16);
      op1lo := op1(15 DOWNTO 0);
      op2hi := op2(31 DOWNTO 16);
      op2lo := op2(15 DOWNTO 0);
      pl := addshift_multiply(op1lo, op2lo);
      pm1 := addshift_multiply(op1hi, op2lo);
      pm2 := addshift_multiply(op1lo, op2hi);
      ph := addshift_multiply(op1hi, op2hi);
      tmpm := unsigned('0'&pm1) + unsigned('0'&pm2);
      reslo := unsigned('0' & tmpm(15 DOWNTO 0) & ZEROS & '0') + unsigned('0' & pl);
      IF reslo(32) = '0' THEN
         reshi := ph + unsigned(ZEROS & tmpm(32 DOWNTO 16));
      ELSE
         reshi := ph + 1 + unsigned(ZEROS & tmpm(32 DOWNTO 16));
      END IF;
      RETURN reshi & reslo(31 DOWNTO 0);
   END multiply_separate;

END PACKAGE BODY mult_instr;