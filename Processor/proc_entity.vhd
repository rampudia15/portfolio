LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

LIBRARY work;
USE work.processor_types.ALL;

ENTITY processor IS
   PORT (
      d_busout	: OUT bit32;
      d_busin	: IN bit32;
      a_bus	: OUT bit32;
      write	: OUT std_ulogic;
      read	: OUT std_ulogic;
      ready	: IN std_ulogic;
      clk	: IN std_ulogic;
      reset	: IN std_ulogic
   );
END processor;
