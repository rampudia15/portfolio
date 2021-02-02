LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

LIBRARY work;
USE work.processor_types.ALL;
USE work.alu_opcode.ALL;

ENTITY controller IS
   PORT (
      clk		: IN std_ulogic;
      op_code		: IN bit6;
      func_code		: IN bit6;
      ready		: IN std_ulogic;
      reset		: IN std_ulogic;

      control_bus	: OUT std_logic_vector(11 downto 0)
   );
END controller;

ARCHITECTURE behaviour OF controller IS
BEGIN
   PROCESS
      ALIAS init	: std_ulogic IS control_bus(11);
      ALIAS reg_dst	: std_ulogic IS control_bus(10);
      ALIAS branch	: std_ulogic IS control_bus(9);
      ALIAS mem_read	: std_ulogic IS control_bus(8);
      ALIAS mem_to_reg	: std_ulogic IS control_bus(7);
      ALIAS mem_write	: std_ulogic IS control_bus(6);
      ALIAS reg_write	: std_ulogic IS control_bus(5);
      ALIAS alu_src	: std_ulogic IS control_bus(4);
      ALIAS alu_op	: bit4 IS control_bus(3 DOWNTO 0);

      VARIABLE instr	: bit6;

   BEGIN
      rst: LOOP
         IF reset = '1' THEN
            control_bus <= (OTHERS => '0');
            instr := (OTHERS => '0');
            WAIT UNTIL rising_edge(clk);
         ELSE
            ready_loop: LOOP
               init <= '0';
               EXIT WHEN ready = '1';
               WAIT UNTIL rising_edge(clk);
               EXIT rst WHEN reset = '1';
            END LOOP ready_loop;
            instr := op_code;
            control_bus <= (OTHERS => '0');
            init <= '1'; -- Notify datapath
            WAIT UNTIL rising_edge(clk);
            EXIT rst WHEN reset = '1';
            init <= '0';
            CASE instr IS
               WHEN op_rtype =>
                  reg_dst <= '1';
                  reg_write <= '1';
                  CASE func_code IS
                     WHEN func_and => alu_op <= alu_and;
                     WHEN func_or => alu_op <= alu_or;
                     WHEN func_add => alu_op <= alu_add;
                     WHEN func_sub => alu_op <= alu_sub;
                     WHEN func_div => alu_op <= alu_div;
                     WHEN func_mult => alu_op <= alu_mult;
                     WHEN func_mflo => alu_op <= alu_mflo;
                     WHEN func_mfhi => alu_op <= alu_mfhi;
                     WHEN func_slt => alu_op <= alu_slt;
                     WHEN func_nop => alu_op <= alu_nop;
                     WHEN OTHERS => control_bus <= (OTHERS => '0');
                  END CASE;
               WHEN op_ori =>
                  reg_write <= '1';
                  alu_src <= '1';
                  alu_op <= alu_ori;
               WHEN op_addi =>
                  reg_write <= '1';
                  alu_src <= '1';
                  alu_op <= alu_add;
               WHEN op_lw =>
                  reg_write <= '1';
                  mem_read <= '1';
                  mem_to_reg <= '1';
                  alu_src <= '1';
                  alu_op <= alu_add;
               WHEN op_sw =>
                  mem_write <= '1';
                  alu_src <= '1';
                  alu_op <= alu_add;
               WHEN op_lui =>
                  reg_write <= '1';
                  alu_src <= '1';
                  alu_op <= alu_lui;
               WHEN op_beq =>
                  branch <= '1';
                  alu_op <= alu_beq;
               WHEN op_bgez =>
                  branch <= '1';
                  alu_op <= alu_bgez;
               WHEN OTHERS => 
            END CASE;
            WAIT UNTIL rising_edge(clk);
            EXIT rst WHEN reset = '1';
         END IF;
      END LOOP rst;
   END PROCESS;
END behaviour;
