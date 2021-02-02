library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

LIBRARY work;
USE work.processor_types.ALL;

ENTITY testset IS
   PORT (
      clk	: OUT std_ulogic
   );
END testset;


ARCHITECTURE set OF testset IS
   CONSTANT period: time := 20 ns;
BEGIN

   -- Clock process definition
   clock_process : PROCESS
   BEGIN
      clk <= '0';
      WAIT FOR period / 2;
      clk <= '1';
      WAIT FOR period/2;
   END PROCESS;

END set;