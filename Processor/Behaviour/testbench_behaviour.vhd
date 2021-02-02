library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

LIBRARY work;
USE work.processor_types.ALL;

entity testbench_behaviour is
end testbench_behaviour;

ARCHITECTURE structure_behaviour OF testbench_behaviour IS

   COMPONENT testset
   PORT (
      clk	: OUT std_ulogic
   );
   END COMPONENT;

   COMPONENT processor
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
   END COMPONENT;

   COMPONENT memory
   PORT (
      d_busout : OUT bit32;
      d_busin  : IN  bit32;
      a_bus    : IN  bit32;
      clk      : IN  std_ulogic;
      write    : IN  std_ulogic;
      read     : IN  std_ulogic;
      ready    : OUT std_ulogic
   );
   END COMPONENT;

-- Local connections
   signal lclk		: std_ulogic;
   signal lreset	: std_ulogic;
   signal ld_busout	: bit32;
   signal ld_busin	: bit32;
   signal la_bus	: bit32;
   signal lwrite	: std_ulogic;
   signal lread		: std_ulogic;
   signal lready	: std_ulogic;

BEGIN

ts : testset
   PORT MAP (
      clk => lclk
   );

behaviour : processor
   PORT MAP (
      d_busout => ld_busout,
      d_busin => ld_busin,
      a_bus => la_bus,
      clk => lclk,
      write => lwrite,
      read => lread,
      ready => lready,
      reset => lreset
   );

mem : memory
   PORT MAP (
      d_busout => ld_busin,
      d_busin => ld_busout,
      a_bus => la_bus,
      clk => lclk,
      write => lwrite,
      read => lread,
      ready => lready
   );

END structure_behaviour;

configuration config_behaviour of testbench_behaviour is
   for structure_behaviour
      for behaviour : processor
         use entity work.processor(behaviour);
      end for;
   end for;
end config_behaviour;
