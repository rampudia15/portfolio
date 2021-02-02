LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

LIBRARY work;
USE work.processor_types.ALL;
USE work.tools.ALL;
USE work.memory_config.ALL;

ENTITY test_env_dpth_ctrl IS
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
END test_env_dpth_ctrl;

ARCHITECTURE structure OF test_env_dpth_ctrl IS
BEGIN
   PROCESS

      PROCEDURE parse_ctrl (data : bit32) IS
      BEGIN
         ctrl_op_code <= data(31 DOWNTO 26);
         ctrl_func_code <= data(5 DOWNTO 0);
         ctrl_ready <= '1';

         dpth_rs <= data(25 DOWNTO 21);
         dpth_rt <= data(20 DOWNTO 16);
         dpth_rd <= data(15 DOWNTO 11);
         dpth_imm <= data(15 DOWNTO 0);
      END parse_ctrl;

   BEGIN

      dpth_ready <= '0';
      WAIT UNTIL clk = '0';

      IF behav_read = '1' AND dpth_read = '1' THEN
         IF behav_a_bus = dpth_a_bus THEN -- Check for difference
            mem_a_bus <= dpth_a_bus;
            mem_write <= '0';
            mem_read <= '1';
            LOOP
               EXIT WHEN mem_ready = '1';
               WAIT UNTIL clk = '0';
            END LOOP;
            IF to_integer(unsigned(dpth_a_bus)) < data_base_address THEN -- Read instruction
               behav_d_busin <= mem_d_busout;
               parse_ctrl(mem_d_busout); -- Send to controller & datapath
               behav_ready <= mem_ready;
               mem_read <= '0';
               LOOP
                  EXIT WHEN mem_ready = '0';
                  WAIT UNTIL clk = '0';
               END LOOP;
               ctrl_ready <= mem_ready;
               behav_ready <= mem_ready;
            ELSE -- Read data
               behav_d_busin <= mem_d_busout;
               dpth_d_busin <= mem_d_busout;
               behav_ready <= mem_ready;
               dpth_ready <= '1';
               mem_read <= '0';
               LOOP
                  EXIT WHEN mem_ready = '0';
                  WAIT UNTIL clk = '0';
               END LOOP;
               dpth_ready <= mem_ready;
               behav_ready <= mem_ready;
            END IF;
            mem_a_bus <= dontcare;
         ELSE -- Error
            error_flag <= '1';
            ASSERT false REPORT "Error Read, not the same address: " & to_hstr(behav_a_bus) & " != " & to_hstr(dpth_a_bus) SEVERITY failure;
         END IF;
      END IF;

      IF behav_write = '1' AND dpth_write = '1' THEN -- Write
         IF behav_a_bus = dpth_a_bus THEN -- Check for difference
            IF behav_d_busout = dpth_d_busout THEN -- Check for difference
               mem_a_bus <= dpth_a_bus;
               mem_d_busin <= dpth_d_busout;
               mem_write <= '1';
               mem_read <= '0';
               LOOP
                  EXIT WHEN mem_ready = '1';
                  WAIT UNTIL clk = '0';
               END LOOP;
               behav_ready <= mem_ready;
               dpth_ready <= mem_ready;
               mem_write <= '0';
               LOOP
                  EXIT WHEN mem_ready = '0';
                  WAIT UNTIL clk = '0';
              END LOOP;
               mem_a_bus <= dontcare;
               mem_d_busin <= dontcare;
               behav_ready <= mem_ready;
               dpth_ready <= mem_ready;
            ELSE -- Error
               error_flag <= '1';
               ASSERT false REPORT "Error Write, not the same data: " & to_hstr(behav_d_busout) & " != " & to_hstr(dpth_d_busout) SEVERITY failure;
            END IF;
         ELSE -- Error
            error_flag <= '1';
            ASSERT false REPORT "Error Write, not the same address: " & to_hstr(behav_a_bus) & " != " & to_hstr(dpth_a_bus) SEVERITY failure;
         END IF;
      END IF;

   END PROCESS;
END structure;