LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

LIBRARY work;
USE work.processor_types.ALL;
USE work.sub_instr.ALL;
USE work.tools.ALL;

PACKAGE div_instr IS
   CONSTANT nbits : integer := 32;

   -- types needed for radix-2 signed-digit representation (SD2)
   TYPE SD2digit IS (zero, neg_one, pos_one);
   TYPE SD2rep IS ARRAY (natural RANGE <>) of SD2digit;
   SUBTYPE SD2rep2N IS SD2rep ((2 * nbits)-1 DOWNTO 0);
   TYPE SD2rep2N_collection IS ARRAY (natural RANGE<>) OF SD2rep2N;

   -- various div functions
   FUNCTION div_repeated_sub (rs:IN bit32; rt:IN bit32) RETURN bit64;
   FUNCTION div_long (rs:IN bit32; rt:IN bit32) RETURN std_logic_vector;
   FUNCTION div_takagi (X:IN bit32; Y:IN bit32) RETURN bit64;

   -- functions used by Takagi
   FUNCTION toSD2(bitvector: in bit32) return SD2rep2N;
   FUNCTION fromSD2(sd2number: in SD2rep2N) return bit32;
   FUNCTION SGNABS(A: IN SD2rep2N) return SD2rep;
   FUNCTION subSD2(A: IN SD2rep2N; T: IN bit64; lsborrow: IN std_logic) return SD2rep2N;
   FUNCTION negSD2(A : in SD2rep2N) return SD2rep2N;

END div_instr;


PACKAGE BODY div_instr IS

   -- not (really) usable in hardware, may use small area but has runtime of Theta(q).
   -- That is dividing 1000 by 2 yields 500 subtractions and those are _small_ numbers in the scheme of 2^31. 
   FUNCTION div_repeated_sub (rs:IN bit32; rt:IN bit32) RETURN bit64 IS
      VARIABLE q : bit32 := (OTHERS => '0');
      VARIABLE r : bit32 := (OTHERS => '0');
      VARIABLE result : bit64 := (OTHERS => '0');
      VARIABLE N : bit32;
      VARIABLE D : bit32;
      VARIABLE flip_q : bit := '0';
      VARIABLE flip_r : bit := '0';
   BEGIN
      D := rt;
      N := rs;

      IF signed(D) = 0 THEN
         RETURN (OTHERS => '0'); -- q = 0, set DIV by 0 flag
      ELSIF D = X"80000000" OR N = X"80000000" THEN
         RETURN (OTHERS => '0'); -- division with minimal number not supported currently
      ELSIF signed(D) < 0 THEN	-- divisor < 0
         -- (r,q) = div(rs,-rt), return (r, -q)
         D := bit32(-signed(D));
         flip_q := '1';	-- is 0, just set to 1
      END IF;
      IF signed(N) < 0 THEN	-- denominator < 0
         -- (r,q) = div(-rs,rt), return (-r,-q))
         N := bit32(-signed(N));
         flip_q := NOT flip_q;	-- might be 0 or 1 so invert it
         flip_r := '1';	-- is 0, just set to 1
      END IF;

      -- both divisor and denominator > 0
      -- do actual division by repeating subtraction until r < rt
      q := (OTHERS => '0');
      r := N;
      WHILE signed(r) >= signed(D) LOOP
         q := bit32(signed(q)+1);
         r := bit32(signed(r)-signed(D));
      END LOOP;

      -- fix r and q to be of right sign
      IF flip_q = '1' THEN
         q := bit32(-signed(q));
      END IF;
      IF flip_r = '1' THEN
         r := bit32(-signed(r));
      END IF;

      result := r & q;
      RETURN result;
   END div_repeated_sub;

--==============================================================================

   FUNCTION div_long (rs:IN bit32; rt:IN bit32) RETURN std_logic_vector IS
      VARIABLE q : bit32 := (OTHERS => '0');
      VARIABLE r : bit32 := (OTHERS => '0');
      VARIABLE tmp : bit32 := (OTHERS => '0');
      VARIABLE result : bit64 := (OTHERS => '0');
      VARIABLE N : bit32;
      VARIABLE D : bit32;
      VARIABLE flip_q : bit := '0';
      VARIABLE flip_r : bit := '0';
      VARIABLE N_mn : bit := '0';
      VARIABLE D_mn : bit := '0';
   BEGIN
      D := rt;
      N := rs;

      IF signed(D) = 1 THEN 
         RETURN (X"00000000" & N); -- DIV by 1 equal denominator
      END IF;

      IF signed(N) = 0 THEN
         RETURN result;
      ELSIF signed(N) < 0 THEN	-- denominator < 0
         -- (r,q) = div(-rs,rt), return (-r,-q))
         IF N = X"80000000" THEN -- Most negative value
            N_mn := '1';
            N := NOT N;
         ELSE
            N := std_logic_vector(1 + signed(not(N))); -- => 2's complement
         END IF;
         flip_q := NOT flip_q;	-- invert
         flip_r := '1';	-- set to 1
      END IF;
      IF signed(D) < 0 THEN	-- divisor < 0
         -- (r,q) = div(rs,-rt), return (r, -q)
         IF D = X"80000000" THEN -- Most negative value
            D_mn := '1';
            IF N_mn = '0' THEN
               D := X"00000001";
            ELSE
               D := NOT D;
            END IF;
         ELSE
            D := std_logic_vector(1 + signed(not(D))); -- => 2's complement
         END IF;
         flip_q := NOT flip_q;	-- invert
      END IF;

      -- both divisor and denominator > 0 so do division
      q := (OTHERS => '0');
      r := (OTHERS => '0');
      FOR i IN 31 DOWNTO 0 LOOP
         r := r(30 DOWNTO 0) & N(i);  -- Shift left | add N(i)
         IF (signed(r) >= signed(D)) THEN
            r := sub_algo1(r, D); -- Sub D from r
            q(i) := '1';
         END IF;
      END LOOP;


      IF D_mn = '1' AND N_mn = '0' THEN -- Invert quotient and remainder when D is most negative integer
         tmp := q;
         q := r;
         r := tmp;
      END IF;

      -- fix r and q to be of right sign
      IF flip_q = '1' THEN
         IF N_mn = '1' AND D_mn = '1' THEN
            q := NOT q;
         ELSE
            q := std_logic_vector(1 + signed(not(q))); -- => 2's complement
         END IF;
      END IF;

      IF flip_r = '1' THEN
         IF N_mn = '1' AND D_mn = '0' THEN
            r := NOT r;
         ELSE
            r := std_logic_vector(1 + signed(not(r))); -- => 2's complement
         END IF;
      END IF;

      result := r & q;
      RETURN result;
   END div_long;

--==============================================================================

   --		A Hardware Algorithm for Integer Division
   --	by Naofumi Takagi, Shunsuke Kadowaki and Kazuyoshi Takagi
   -- 			July 2005		DOI: 10.1109/ARITH.2005.6

   -- X = Y * Z + R (X and Y are inputs, Z and R outputs)
   -- ASSUMPTION: -2^(n-1) < X,Y < 2^(n-1) where n = wordsize = 32.
   -- 		  So both X and Y are valid 32 bit values other than the minimal value.
   FUNCTION div_takagi (X:IN bit32; Y:IN bit32) RETURN bit64 IS
      VARIABLE Z : bit32 := (OTHERS => '0');
      VARIABLE R : bit32 := (OTHERS => '0');
      VARIABLE D : bit32 := (Others => '0');
      VARIABLE P : bit32 := (OTHERS => '0');
      ALIAS y_msb : std_logic IS Y(31); -- y_n-1, the most significant bit of Y
      ALIAS x_msb : std_logic IS X(31); -- x_n-1, the most significant bit of X
      VARIABLE Rhat : SD2rep2N_collection (0 TO nbits);
      VARIABLE Rhat_abs : SD2rep2N_collection (0 TO nbits);
      VARIABLE Rhat_sgn : SD2rep (0 TO nbits);
      VARIABLE Rhat0star_abs : SD2rep2N;
      VARIABLE Rhat0star_sgn : SD2digit;
      VARIABLE signR : SD2rep (0 TO nbits);
      VARIABLE SGNABS_ret_val : SD2rep (2*nbits DOWNTO 0); -- stores return value of the sgnabs function
         ALIAS SGN_ret_val : SD2digit IS SGNABS_ret_val(2*nbits);
         ALIAS ABS_ret_val : SD2rep2N IS SGNABS_ret_val(2*nbits-1 DOWNTO 0);
      VARIABLE result : bit64 := (OTHERS => '0');
   BEGIN

      -- line one of IDIV-HA : use positive divider, compenstation will be done later
      IF y_msb = '0' THEN
         D := Y;
      ELSE
         D := NOT Y;
      END IF;

      -- lines 2 to 5 of IDIV-HA : prepare first partial remainder and sign bookkeeping
      Rhat(nbits) := toSD2(X);
      IF x_msb = '1' THEN
         signR(nbits) := neg_one;
      ELSE
         signR(nbits) := pos_one;
      END IF;

      -- lines 6 to 18 : loop over all partial remainders
      FOR j IN nbits-1 DOWNTO 0 LOOP
         -- lines 7 to 11 : check if partial quotient should be added to the quotient
         IF signR(j+1) = neg_one THEN
            IF y_msb = '0' THEN
               P(j+1) := '0';
            ELSE
               P(j+1) := '1';
            END IF;
         ELSE
            IF y_msb = '0' THEN
               P(j+1) := '1';
            ELSE
               P(j+1) := '0';
            END IF;
         END IF;

         -- line 12 of IDIV-HA : Subtract the partial quotient from the partial remainder.
         -- extension of D can be unsigned since D is guaranteed to be positive
         --pass y_msb as borrow bit since we only need correction (borrow bit 1) when y_msb = 1 (when Y < 0)
         Rhat(j) := subSD2(Rhat_abs(j+1), bit64(resize(unsigned(D), bit64'LENGTH) SLL j), y_msb);

         -- use SGNABS function to obtain both sign and absolute of
         SGNABS_ret_val := SGNABS(Rhat(j));
         -- lines 13 to 17 :
         Rhat_abs(j) := ABS_ret_val;
         Rhat_sgn(j) := SGN_ret_val;
         IF SGN_ret_val = neg_one OR (Rhat_sgn(j) = zero AND signR(j+1) = neg_one) THEN
            --	sign(Rj) := -sign(Rj+1)
            IF signR(j+1) = neg_one THEN
               signR(j) := pos_one;
            ELSIF signR(j+1) = pos_one THEN
               signR(j) := neg_one;
            ELSE
               signR(j) := zero;
            END IF;
         ELSE
            signR(j) := signR(j+1);
         END IF;
      END LOOP;

      -- line 19 of IDIV-HA : Do final update of quotient (like lines 7 to 11 but subtraction always happened)
      P(0) := '1';

      -- Calculate Rhat_0_star and its sign and absolute, toss Rhat_0_star itself since never used again
      -- extension of D can be unsigned since D is guaranteed to be positive
      -- pass y_msb as borrow bit since we only need correction (borrow bit 1) when y_msb = 1 (when Y < 0)
      SGNABS_ret_val := SGNABS(subSD2(Rhat_abs(0), bit64(resize(unsigned(D), bit64'LENGTH)), y_msb));
      Rhat0star_abs := ABS_ret_val;
      Rhat0star_sgn := SGN_ret_val;

      -- lines 20 to 35 of IDIV-HA : Corrections of P into Z and Rhat into R
      IF x_msb = '0' THEN
         -- lines 21 to 25 of IDIV-HA : Correct R if subtraction overshot (R0 < 0 while X >= 0)
         IF signR(0) = neg_one THEN
            R := fromSD2(Rhat0star_abs);
            IF y_msb = '0' THEN
               -- Z := P - 1
               Z := P(31 DOWNTO 1) & '0';
            ELSE
               -- Z := P + 1
               Z := bit32(signed(P) + 1);
            END IF;					
         ELSE
            R := fromSD2(Rhat_abs(0));
            Z := P;
         END IF;
      ELSE
         IF Rhat0star_sgn = zero THEN
            R := fromSD2(negSD2(Rhat0star_abs));
            IF y_msb = '0' THEN
               -- Z := P - 1
               Z := P(31 DOWNTO 1) & '0';
            ELSE
               -- Z := P + 1
               Z := bit32(signed(P) + 1);
            END IF;
         ELSIF Rhat_sgn(0) /= zero AND signR(0) = pos_one THEN
            R := fromSD2(negSD2(Rhat0star_abs));
            IF y_msb = '0' THEN
               -- Z := P + 1
               Z := bit32(signed(P) + 1);
            ELSE
               -- Z := P - 1
               Z := P(31 DOWNTO 1) & '0';
            END IF;
         ELSE
            R := fromSD2(negSD2(Rhat_abs(0)));
            Z := P;
         END IF;
      END IF;

      result := R & Z;
      RETURN result;
   END div_takagi;


   -- BELOW ARE AUXILIARY FUNCTIONS NEEDED FOR THE TAKAGI DIVIDER


   -- Negates the SD2 integer
   FUNCTION negSD2(A : in SD2rep2N) return SD2rep2N IS
      VARIABLE negation : SD2rep2N := (OTHERS => zero);
   BEGIN
      FOR i IN A'RANGE LOOP
         CASE A(i) IS
            WHEN neg_one =>
               negation(i) := pos_one;
            WHEN pos_one =>
               negation(i) := neg_one;
            WHEN OTHERS => NULL; -- no need for action when zero since negation starts as all zero
         END CASE;
      END LOOP;
		
      RETURN negation;
   END negSD2;
	

   -- Subtract T from A as per Section II.C. Works without borrow propagation and uses lookuptables.
   -- T is a two's complement n-bit vector and A is a n-digit SD2 integer
   -- lsborrow is the borrow bit to be used for the least significant digit.
   FUNCTION subSD2(A: IN SD2rep2N; T: IN bit64; lsborrow: IN std_logic) return SD2rep2N IS
      VARIABLE borrow_bits : std_logic_vector (2*nbits DOWNTO 0);
      VARIABLE difference_bits : bit64;
      VARIABLE result : SD2rep2N;
   BEGIN
		
      -- First determine per digit the borrow and intermediate difference
      FOR i IN 2*nbits-1 DOWNTO 0 LOOP
         -- Convert nested case to 2D LUT?
         CASE A(i) IS
            WHEN neg_one =>
               CASE T(i) IS
                  WHEN '0' =>
                     borrow_bits(i+1) := '1';
                     difference_bits(i) := '1';
                  WHEN '1' =>
                     borrow_bits(i+1) := '1';
                     difference_bits(i) := '0';
                  WHEN OTHERS => REPORT "SD2 subtract: Bits in vector T not 0 or 1" SEVERITY failure;
               END CASE;
            WHEN zero =>
               CASE T(i) IS
                  WHEN '0' =>
                     borrow_bits(i+1) := '0';
                     difference_bits(i) := '0';
                  WHEN '1' =>
                     borrow_bits(i+1) := '1';
                     difference_bits(i) := '1';
                  WHEN OTHERS => REPORT "SD2 subtract: Bits in vector T not 0 or 1" SEVERITY failure;
               END CASE;
            WHEN pos_one =>
               CASE T(i) IS
                  WHEN '0' =>
                     borrow_bits(i+1) := '0';
                     difference_bits(i) := '1';
                  WHEN '1' =>
                     borrow_bits(i+1) := '0';
                     difference_bits(i) := '0';
                  WHEN OTHERS => REPORT "SD2 subtract: Bits in vector T not 0 or 1" SEVERITY failure;
               END CASE;
         END CASE;
      END LOOP;

      -- Then determine final difference: the result
      FOR i IN A'RANGE LOOP
         CASE (std_logic_vector'(borrow_bits(i) & difference_bits(i))) IS
            WHEN "01" =>
               result(i) := pos_one;
            WHEN "10" =>
               result(i) := neg_one;
            WHEN OTHERS =>
               result(i) := zero;
         END CASE;
      END LOOP;

      RETURN result;
   END subSD2;
	

	-- Calculates both the sign and the absolute value of A. Returns the values as as n+1 digit SD2rep, first digit is sign rest is the abolute value
	FUNCTION SGNABS(A: IN SD2rep2N) return SD2rep IS
		VARIABLE S : SD2rep (nbits DOWNTO 0);
		VARIABLE B : SD2rep2N;
		VARIABLE result: SD2rep (2*nbits DOWNTO 0);
		ALIAS sgn : SD2digit IS result(2*nbits);
		ALIAS abs_val : SD2rep2N IS result (2*nbits-1 DOWNTO 0);
	BEGIN
		S(nbits) := zero;

		FOR i IN nbits-1 DOWNTO 0 LOOP
			IF S(i+1) = zero AND A(i) = neg_one THEN
				S(i) := neg_one;
			ELSIF S(i+1) = zero AND A(i) = pos_one THEN
				S(i) := pos_one;
			ELSE
				S(i) := S(i+1);
			END IF;

			IF S(i) = neg_one AND A(i) = neg_one THEN
				B(i) := pos_one;
			ELSIF S(i) = neg_one AND A(i) = pos_one THEN
				B(i) := neg_one;
			ELSE
				B(i) := A(i);
			END IF;
		END LOOP;

		sgn := S(0);
		abs_val := B;
		RETURN result;
	END SGNABS;
	

	-- Converts from 2's complement 32bit std_logic_vector to SD2 representation with 32 digits
	FUNCTION toSD2(bitvector: in bit32) return SD2rep2N IS
		VARIABLE result : SD2rep2N := (OTHERS => zero);
	BEGIN
		-- if negative number
		IF bitvector(nbits-1) = '1' THEN
			result(nbits-1) := neg_one;
		ELSE
			result(nbits-1) := zero;
		END IF;
		-- process other bits
		FOR i IN nbits-2 DOWNTO 0 LOOP
			IF bitvector(i) = '0' THEN
				result(i) := zero;
			ELSE
				result(i) := pos_one;
			END IF;
		END LOOP;

		RETURN result;
	END toSD2;
	

	-- Converts from SD2 representation with 32 digits to 32bit std_logic_vector
	FUNCTION fromSD2(sd2number: in SD2rep2N) return bit32 IS
		VARIABLE result : bit32 := (OTHERS => '0');
		VARIABLE sd2value : integer := 0;
	BEGIN
		-- first calculate value
		FOR i IN 2*nbits-1 DOWNTO 0 LOOP
			IF sd2number(i) = pos_one THEN
				sd2value := sd2value + 2**i;
			ELSIF sd2number(i) = neg_one THEN
				sd2value := sd2value - 2**i;
			END IF;	-- else do nothing, 0 digit needs no add/sub
		END LOOP;

		result := std_logic_vector(to_signed(sd2value, result'length));
		
		RETURN result;		
	END fromSD2;
	
	

--==============================================================================

	


	-- look into following links:
	--	http://crisp.massey.ac.nz/pdfs/2006_ENZCon_206.pdf
	--	https://www.researchgate.net/publication/291832548_Design_and_Analysis_of_Integer_Divider_Using_Non_Restoring_Division_Algorithm
	
END PACKAGE BODY div_instr;
