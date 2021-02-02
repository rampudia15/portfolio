-- File memory_config.vhd
-- This package contains the constants and functions used for the
-- simple memory module used for a subset MIPS processor.
-- The locations (start addresses) of DATA and TEXT segments and the SIZE of these segments are declared.
-- The conversion functions can convert hex string to binary string (and reverse).

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;
PACKAGE memory_config IS
   -- USER can change memory location of data and text segments and the size of these segments.
   CONSTANT text_base_address : natural :=16#400000#;
   CONSTANT text_base_size    : natural := 800; -- each word is 4 bytes
   CONSTANT data_base_address : natural :=16#10010000#;
   CONSTANT data_base_size    : natural := 300;
   -- *******************************
  
   -- convert hex to binary
   FUNCTION hex2bin (hex: character) RETURN std_logic_vector;
   -- convert a string of hex digits to binary
   FUNCTION hexvec2bin (hexv : string) RETURN std_logic_vector;
   -- convert 4 bits binary to character
   FUNCTION bin2hex (bin : std_logic_vector(3 DOWNTO 0)) RETURN character;  
   -- convert binary pattern to hex string (input is zero extended on left to multiple of 4)
   FUNCTION binvec2hex (bin : std_logic_vector) RETURN string;
  
END memory_config;

PACKAGE BODY memory_config IS
   FUNCTION hex2bin (hex: character) RETURN std_logic_vector IS
      VARIABLE result : std_logic_vector (3 DOWNTO 0);
   BEGIN
      CASE hex IS
         WHEN '0' =>     result := "0000";
         WHEN '1' =>     result := "0001";
         WHEN '2' =>     result := "0010";
         WHEN '3' =>     result := "0011";
         WHEN '4' =>     result := "0100";
         WHEN '5' =>     result := "0101";
         WHEN '6' =>     result := "0110";
         WHEN '7' =>     result := "0111";
         WHEN '8' =>     result := "1000";
         WHEN '9' =>     result := "1001";
         WHEN 'A'|'a' => result := "1010";
         WHEN 'B'|'b' => result := "1011";
         WHEN 'C'|'c' => result := "1100";
         WHEN 'D'|'d' => result := "1101";
         WHEN 'E'|'e' => result := "1110";
         WHEN 'F'|'f' => result := "1111";
         WHEN 'X'|'x' => result := "XXXX";
         WHEN OTHERS =>  NULL;
      END CASE;
      RETURN result;
   END;  
  
   FUNCTION hexvec2bin (hexv : string) RETURN std_logic_vector IS
      CONSTANT hv : string(hexv'LENGTH DOWNTO 1):= hexv; -- range xx downto 1, to prevent NULL slice error (now NULL slice is "0 DOWNTO 1")
   BEGIN
      IF hexv'LENGTH /= 0 THEN
         RETURN hex2bin(hv(hv'LEFT)) & hexvec2bin (hv(hv'LENGTH-1 DOWNTO 1));
      ELSE
         RETURN "";
      END IF;
   END hexvec2bin;  
  
   FUNCTION bin2hex (bin : std_logic_vector(3 DOWNTO 0)) RETURN character IS
      TYPE lut_tp IS ARRAY (0 TO 15) of character;
      CONSTANT lut : lut_tp := ('0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F');
   BEGIN
      RETURN lut(to_integer(unsigned(bin)));
   END bin2hex;
  
   -- local function.
   -- If size of 'a' is not a multiple of 4, it is zero extended on the left to a multiple of 4 bits
   FUNCTION multiple_of_4 (a : std_logic_vector) RETURN std_logic_vector IS
   BEGIN
      IF (a'LENGTH REM 4)/=0 THEN
         RETURN (3 DOWNTO (a'LENGTH REM 4)=>'0') & a;
      ELSE
         RETURN a;
      END IF;
   END multiple_of_4;
  
   -- local function: function binary vector to hex vector. Input length must be multiple of 4.
   FUNCTION binvec2hex_multiple4 (bin : std_logic_vector) RETURN string IS
      CONSTANT binv : std_logic_vector(0 TO bin'LENGTH-1) := bin; -- align index
   BEGIN
      IF binv'LENGTH=0 THEN
         RETURN "";
      ELSIF binv'LENGTH=4 THEN
         RETURN bin2hex(binv(0 TO 3)) & ""; -- &"" used to convert character to string
      ELSE
         RETURN bin2hex(binv(0 TO 3)) & binvec2hex_multiple4(binv(4 TO binv'LENGTH-1));
      END IF;
   END binvec2hex_multiple4;
  
   FUNCTION binvec2hex (bin : std_logic_vector) RETURN string IS
   BEGIN
      RETURN binvec2hex_multiple4(multiple_of_4(bin));
   END binvec2hex;
  
END memory_config;
