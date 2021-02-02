LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

PACKAGE tools IS

   -- Op code to string
   FUNCTION op_to_string (inp:IN std_logic_vector) RETURN string;
   -- Vector to hexadecimal string
   FUNCTION to_hstr (inp:IN std_logic_vector) RETURN string;
  
END tools;

PACKAGE BODY tools IS

   FUNCTION op_to_string (inp:IN std_logic_vector) RETURN string IS
      VARIABLE str : string(1 TO inp'LENGTH);
      VARIABLE inpi : std_logic_vector(str'RANGE) := inp; -- align vector with result string

   BEGIN
      FOR i in str'RANGE LOOP
         CASE inpi(i) IS
            WHEN 'U'=> str(i):='U';
            WHEN 'X'=> str(i):='X';          
            WHEN '0'=> str(i):='0';          
            WHEN '1'=> str(i):='1';          
            WHEN 'Z'=> str(i):='Z';          
            WHEN 'W'=> str(i):='W';          
            WHEN 'L'=> str(i):='L';          
            WHEN 'H'=> str(i):='H';          
            WHEN '-'=> str(i):='-';          
         END CASE;
      END LOOP;
      RETURN str;
   END op_to_string;

   FUNCTION to_hstr (inp:IN std_logic_vector) RETURN string IS
      VARIABLE str : string(1 TO 8);
      VARIABLE inpi : std_logic_vector(3 DOWNTO 0);
   
   BEGIN
      FOR i in str'RANGE LOOP
         inpi := inp(31-((i-1)*4) DOWNTO 28-((i-1)*4));
         CASE to_integer(signed(inpi)) IS
            WHEN 0 => str(i) := '0';
            WHEN 1 => str(i) := '1';
            WHEN 2 => str(i) := '2';
            WHEN 3 => str(i) := '3';
            WHEN 4 => str(i) := '4';
            WHEN 5 => str(i) := '5';
            WHEN 6 => str(i) := '6';
            WHEN 7 => str(i) := '7';
            WHEN 8 => str(i) := '8';
            WHEN 9 => str(i) := '9';
            WHEN 10 => str(i) := 'A';
            WHEN 11 => str(i) := 'B';
            WHEN 12 => str(i) := 'C';
            WHEN 13 => str(i) := 'D';
            WHEN 14 => str(i) := 'E';
            WHEN 15 => str(i) := 'F';
            WHEN OTHERS => NULL;
         END CASE;
      END LOOP;
      RETURN str;
   END to_hstr;

END PACKAGE BODY tools;