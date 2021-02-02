library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

LIBRARY work;
USE work.processor_types.ALL;
USE work.alu_opcode.ALL;

entity testbench_dpth_ctrl is
end testbench_dpth_ctrl;

ARCHITECTURE structure_dpth_ctrl OF testbench_dpth_ctrl IS

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

   COMPONENT datapath
   PORT (
      clk		: IN std_ulogic;
      rs,rt,rd		: IN bit5; -- Registers to read/write
      imm		: IN bit16; -- Immediate value
      control_bus	: IN std_logic_vector(11 downto 0);
      a_bus		: OUT bit32; -- Address of next instruction or data to read
      d_busin		: IN bit32; -- Data read
      d_busout		: OUT bit32; -- Data to write
      write		: OUT std_ulogic; -- Processor is ready to write data
      read		: OUT std_ulogic; -- Processor is ready to read next instruction or data
      ready		: IN std_ulogic -- Data from memory is ready
   );
   END COMPONENT;

   COMPONENT controller
   PORT (
      clk		: IN std_ulogic;
      op_code		: IN bit6;
      func_code		: IN bit6;
      ready		: IN std_ulogic;
      reset		: IN std_ulogic;
      control_bus	: OUT std_logic_vector(11 downto 0)
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

   COMPONENT test_env_dpth_ctrl
   PORT (
      clk		: IN std_ulogic;
      error_flag	: OUT std_ulogic;

      mem_d_busin	: OUT bit32;
      mem_d_busout	: IN bit32;
      mem_a_bus		: OUT bit32;
      mem_write		: OUT std_ulogic;
      mem_read		: OUT std_ulogic;
      mem_ready		: IN std_ulogic;

      behav_d_busout	: IN bit32;
      behav_d_busin	: OUT bit32;
      behav_a_bus	: IN bit32;
      behav_write	: IN std_ulogic;
      behav_read	: IN std_ulogic;
      behav_ready	: OUT std_ulogic;

      dpth_rs		: OUT bit5;
      dpth_rt		: OUT bit5;
      dpth_rd		: OUT bit5;
      dpth_imm		: OUT bit16;
      dpth_a_bus	: IN bit32;
      dpth_d_busout	: IN bit32;
      dpth_d_busin	: OUT bit32;
      dpth_write	: IN std_ulogic;
      dpth_read		: IN std_ulogic;
      dpth_ready	: OUT std_ulogic;

      ctrl_op_code	: OUT bit6;
      ctrl_func_code	: OUT bit6;
      ctrl_ready	: OUT std_ulogic
   );
   END COMPONENT;

-- Local connections
   signal lclk			: std_ulogic;
   signal lreset		: std_ulogic;
   signal lerror_flag		: std_ulogic;

   signal lmem_d_busin		: bit32;
   signal lmem_d_busout		: bit32;
   signal lmem_a_bus		: bit32;
   signal lmem_write		: std_ulogic;
   signal lmem_read		: std_ulogic;
   signal lmem_ready		: std_ulogic;

   signal lbehav_d_busout	: bit32;
   signal lbehav_d_busin	: bit32;
   signal lbehav_a_bus		: bit32;
   signal lbehav_write		: std_ulogic;
   signal lbehav_read		: std_ulogic;
   signal lbehav_ready		: std_ulogic;

   signal ldpth_rs		: bit5;
   signal ldpth_rt		: bit5;
   signal ldpth_rd		: bit5;
   signal ldpth_imm		: bit16;
   signal ldpth_a_bus		: bit32;
   signal ldpth_d_busout	: bit32;
   signal ldpth_d_busin		: bit32;
   signal ldpth_write		: std_ulogic;
   signal ldpth_read		: std_ulogic;
   signal ldpth_ready		: std_ulogic;

   signal lctrl_op_code		: bit6;
   signal lctrl_func_code	: bit6;
   signal lctrl_ready		: std_ulogic;

   signal lcontrol_bus		: std_logic_vector(11 downto 0);

BEGIN

ts : testset
   PORT MAP (
      clk => lclk
   );

behaviour : processor
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

dpth : datapath
   PORT MAP (
      clk => lclk,
      rs => ldpth_rs,
      rt => ldpth_rt,
      rd => ldpth_rd,
      imm => ldpth_imm,
      control_bus => lcontrol_bus,
      a_bus => ldpth_a_bus,
      d_busin => ldpth_d_busin,
      d_busout => ldpth_d_busout,
      write => ldpth_write,
      read => ldpth_read,
      ready => ldpth_ready
   );

ctrl : controller
   PORT MAP (
      clk => lclk,
      op_code => lctrl_op_code,
      func_code => lctrl_func_code,
      control_bus => lcontrol_bus,
      ready => lctrl_ready,
      reset => lreset 
   );

test_env : test_env_dpth_ctrl
   PORT MAP (
      clk => lclk,
      error_flag => lerror_flag,
      mem_d_busin => lmem_d_busin,
      mem_d_busout => lmem_d_busout,
      mem_a_bus => lmem_a_bus,
      mem_write => lmem_write,
      mem_read => lmem_read,
      mem_ready => lmem_ready,
      behav_d_busout => lbehav_d_busout,
      behav_d_busin => lbehav_d_busin,
      behav_a_bus => lbehav_a_bus,
      behav_write => lbehav_write,
      behav_read => lbehav_read,
      behav_ready => lbehav_ready,
      dpth_rs => ldpth_rs,
      dpth_rt => ldpth_rt,
      dpth_rd => ldpth_rd,
      dpth_imm => ldpth_imm,
      dpth_a_bus => ldpth_a_bus,
      dpth_d_busout => ldpth_d_busout,
      dpth_d_busin => ldpth_d_busin,
      dpth_write => ldpth_write,
      dpth_read => ldpth_read,
      dpth_ready => ldpth_ready,
      ctrl_op_code => lctrl_op_code,
      ctrl_func_code => lctrl_func_code,
      ctrl_ready => lctrl_ready
   );

END structure_dpth_ctrl;

configuration config_dpth_ctrl of testbench_dpth_ctrl is
   for structure_dpth_ctrl
      for behaviour : processor
         use entity work.processor(behaviour);
      end for;
      for dpth : datapath
         use entity work.datapath(rtl);
      end for;
      for ctrl : controller
         use entity work.controller(behaviour);
      end for;
      for mem : memory
         use entity work.memory(behaviour);
      end for;
   end for;
end config_dpth_ctrl;

configuration config_dpth_ctrl_exp of testbench_dpth_ctrl is
   for structure_dpth_ctrl
      for behaviour : processor
         use entity work.processor(behaviour);
      end for;
      for dpth : datapath
         use entity work.datapath(rtl);
      end for;
      for ctrl : controller
         use entity work.controller(behaviour);
      end for;
      for mem : memory
         use entity work.memory(exp_program);
      end for;
   end for;
end config_dpth_ctrl_exp;

configuration config_dpth_ctrl_post_sim of testbench_dpth_ctrl is
   for structure_dpth_ctrl
      for behaviour : processor
         use entity work.processor(behaviour);
      end for;
      for dpth : datapath
         use entity work.datapath(structure);
      end for;
      for ctrl : controller
         use entity work.controller(behaviour);
      end for;
      for mem : memory
         use entity work.memory(exp_program);
      end for;
   end for;
end config_dpth_ctrl_post_sim;
