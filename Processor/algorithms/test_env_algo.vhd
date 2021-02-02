LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

LIBRARY work;
USE work.processor_types.ALL;
USE work.tools.ALL;

ENTITY test_env_algo IS
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
END test_env_algo;

ARCHITECTURE structure OF test_env_algo IS
BEGIN

   PROCESS
   BEGIN

      WAIT UNTIL rising_edge(clk);

      IF behav_read = '1' AND algo_read = '1' THEN
         IF behav_a_bus = algo_a_bus THEN -- Check for difference
            mem_a_bus <= behav_a_bus;
            mem_write <= '0';
            mem_read <= '1';
            LOOP
               EXIT WHEN mem_ready = '1';
               WAIT UNTIL clk = '0';
            END LOOP;
            behav_d_busout <= mem_d_busin;
            algo_d_busout <= mem_d_busin;
            behav_ready <= mem_ready;
            algo_ready <= mem_ready;
            mem_read <= '0';
            LOOP
               EXIT WHEN mem_ready = '0';
               WAIT UNTIL clk = '0';
            END LOOP;
            behav_ready <= mem_ready;
            algo_ready <= mem_ready;
         ELSE -- Error
            error_flag <= '1';
            ASSERT false REPORT "Error Read, not the same address: " & to_hstr(behav_a_bus) & " != " & to_hstr(algo_a_bus) SEVERITY failure;
         END IF;
      END IF;

      IF behav_write = '1' AND algo_write = '1' THEN
         IF behav_a_bus = algo_a_bus THEN -- Check for difference
            IF behav_d_busin = algo_d_busin THEN -- Check for difference
               mem_a_bus <= behav_a_bus;
               mem_d_busout <= behav_d_busin;
               mem_read <= '0';
               mem_write <= '1';
               LOOP
                  EXIT WHEN mem_ready = '1';
                  WAIT UNTIL clk = '0';
               END LOOP;
               behav_ready <= mem_ready;
               algo_ready <= mem_ready;
               mem_write <= '0';
               LOOP
                  EXIT WHEN mem_ready = '0';
                  WAIT UNTIL clk = '0';
               END LOOP;
               behav_ready <= mem_ready;
               algo_ready <= mem_ready;
            ELSE -- Error
               error_flag <= '1';
               ASSERT false REPORT "Error Write, not the same data: " & to_hstr(behav_d_busin) & " != " & to_hstr(algo_d_busin) SEVERITY failure;
            END IF;
         ELSE -- Error
            error_flag <= '1';
            ASSERT false REPORT "Error Write, not the same address: " & to_hstr(behav_a_bus) & " != " & to_hstr(algo_a_bus) SEVERITY failure;
         END IF;
      END IF;

   END PROCESS;
END structure;
