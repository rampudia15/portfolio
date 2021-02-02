library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

LIBRARY work;
USE work.processor_types.ALL;

entity testbench_algo is
end testbench_algo;

ARCHITECTURE structure_algo OF testbench_algo IS

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

   COMPONENT test_env_algo
   PORT (
      behav_d_busin	: IN bit32; -- From behav data out
      algo_d_busin	: IN bit32; -- From algo data out
      mem_d_busout	: OUT bit32; -- To mem data in

      behav_d_busout	: OUT bit32; -- To behav data in
      algo_d_busout	: OUT bit32; -- To algo data in
      mem_d_busin	: IN bit32; -- From mem data out

      behav_a_bus	: IN bit32; -- From behav addr bus
      algo_a_bus	: IN bit32; -- From algo addr bus
      mem_a_bus		: OUT bit32; -- To mem addr bus

      behav_write	: IN std_ulogic; -- From behav write
      algo_write	: IN std_ulogic; -- From algo write
      mem_write		: OUT std_ulogic; -- To mem write

      behav_read	: IN std_ulogic; -- From behav read
      algo_read		: IN std_ulogic; -- From algo read
      mem_read		: OUT std_ulogic; -- To mem read

      behav_ready	: OUT std_ulogic; -- To behav ready
      algo_ready	: OUT std_ulogic; -- To algo ready
      mem_ready		: IN std_ulogic; -- From mem ready

      clk		: IN std_ulogic;
      error_flag	: OUT std_ulogic
   );
   END COMPONENT;

-- Common local connections
   signal lclk			: std_ulogic;
   signal lreset		: std_ulogic;
   signal lerror_flag		: std_ulogic;

-- Behaviour local connections
   signal lbehav_d_busout	: bit32;
   signal lbehav_d_busin	: bit32;
   signal lbehav_a_bus		: bit32;
   signal lbehav_write		: std_ulogic;
   signal lbehav_read		: std_ulogic;
   signal lbehav_ready		: std_ulogic;

-- Algorithmic local connections
   signal lalgo_d_busout	: bit32;
   signal lalgo_d_busin		: bit32;
   signal lalgo_a_bus		: bit32;
   signal lalgo_write		: std_ulogic;
   signal lalgo_read		: std_ulogic;
   signal lalgo_ready		: std_ulogic;

-- Memory local connections
   signal lmem_d_busout		: bit32;
   signal lmem_d_busin		: bit32;
   signal lmem_a_bus		: bit32;
   signal lmem_write		: std_ulogic;
   signal lmem_read		: std_ulogic;
   signal lmem_ready		: std_ulogic;


BEGIN

ts : testset
   PORT MAP (
      clk => lclk
   );

behav : processor
   PORT MAP (
      d_busout => lbehav_d_busout,
      d_busin => lbehav_d_busin,
      a_bus => lbehav_a_bus,
      clk => lclk,
      write => lbehav_write,
      read => lbehav_read,
      ready => lbehav_ready,
      reset => lreset
   );

algo : processor
   PORT MAP (
      d_busout => lalgo_d_busout,
      d_busin => lalgo_d_busin,
      a_bus => lalgo_a_bus,
      clk => lclk,
      write => lalgo_write,
      read => lalgo_read,
      ready => lalgo_ready,
      reset => lreset
   );

struct : test_env_algo
   PORT MAP (
      behav_d_busin => lbehav_d_busout,
      algo_d_busin => lalgo_d_busout,
      mem_d_busout => lmem_d_busin,
      behav_d_busout => lbehav_d_busin,
      algo_d_busout => lalgo_d_busin,
      mem_d_busin => lmem_d_busout,
      behav_a_bus => lbehav_a_bus,
      algo_a_bus => lalgo_a_bus,
      mem_a_bus => lmem_a_bus,
      behav_write => lbehav_write,
      algo_write => lalgo_write,
      mem_write => lmem_write,
      behav_read => lbehav_read,
      algo_read => lalgo_read,
      mem_read => lmem_read,
      behav_ready => lbehav_ready,
      algo_ready => lalgo_ready,
      mem_ready => lmem_ready,
      clk => lclk,
      error_flag => lerror_flag
   );

mem : memory
   PORT MAP (
      d_busout => lmem_d_busout,
      d_busin => lmem_d_busin,
      a_bus => lmem_a_bus,
      clk => lclk,
      write => lmem_write,
      read => lmem_read,
      ready => lmem_ready
   );

END structure_algo;

configuration config_algo of testbench_algo is
   for structure_algo
      for behav : processor
         use entity work.processor(behaviour);
      end for;
      for algo : processor
         use entity work.processor(algorithmic);
      end for;
   end for;
end config_algo;
