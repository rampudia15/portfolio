LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

LIBRARY work;
USE work.processor_types.ALL;
USE work.tools.ALL;
USE work.alu_opcode.ALL;
USE work.memory_config.ALL;

USE work.and_instr.ALL;
USE work.or_instr.ALL;
USE work.ori_instr.ALL;
USE work.sub_instr.ALL;
USE work.slt_instr.ALL;
USE work.branch_instr.ALL;
USE work.mult_instr.ALL;
USE work.div_instr.ALL;

ENTITY datapath IS
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
END datapath;

ARCHITECTURE rtl OF datapath IS

   ALIAS init		: std_ulogic IS control_bus(11);
   ALIAS reg_dst	: std_ulogic IS control_bus(10);
   ALIAS branch		: std_ulogic IS control_bus(9);
   ALIAS mem_read	: std_ulogic IS control_bus(8);
   ALIAS mem_to_reg	: std_ulogic IS control_bus(7);
   ALIAS mem_write	: std_ulogic IS control_bus(6);
   ALIAS reg_write	: std_ulogic IS control_bus(5);
   ALIAS alu_src	: std_ulogic IS control_bus(4);
   ALIAS alu_op		: bit4 IS control_bus(3 DOWNTO 0);

   SIGNAL pc		: natural := text_base_address;

   SIGNAL rd_reg1	: bit32 := (OTHERS => '0');
   SIGNAL rd_reg2	: bit32 := (OTHERS => '0');
   SIGNAL wr_reg	: bit5 := (OTHERS => '0');

   SIGNAL alu_res	: bit32 := (OTHERS => '0');
   SIGNAL mem_res	: bit32 := (OTHERS => '0');
   SIGNAL alu_long_res	: bit64 := (OTHERS => '0');
   SIGNAL overflow_flag : std_logic;

   SIGNAL register_file	: registers := (OTHERS => X"00000000");
   SIGNAL register_LO	: bit32 := (OTHERS => '0');
   SIGNAL register_HI	: bit32 := (OTHERS => '0');
   
   SIGNAL state		: bit3 := (OTHERS => '0');
BEGIN   
   MIPS_CYCLE:PROCESS

   VARIABLE wr_data	: bit32 := (OTHERS => '0');
   BEGIN

      WAIT UNTIL rising_edge(clk);
      
      CASE state IS
         WHEN "000" => -- IF1: Ask for next instruction
            REPORT "state IF1" SEVERITY note;
            alu_res <= (OTHERS => '0'); -- Reset
            mem_res <= (OTHERS => '0'); -- Reset
            a_bus <= std_logic_vector(to_unsigned(pc, 32));
            read <= '1';
            IF init = '1' THEN
               read <= '0';
               a_bus <= dontcare; 
               state <= "001"; -- ID
            ELSE
               state <= "000"; -- IF2
            END IF;

         WHEN "001" => -- ID: Handle registers for ALU
            REPORT "state ID" SEVERITY note;
            rd_reg1 <= register_file(to_integer(unsigned(rs)));

            IF alu_src = '0' THEN
               rd_reg2 <= register_file(to_integer(unsigned(rt)));
            ELSE
               IF imm(15) = '0' THEN -- Sign extension
                  rd_reg2 <= (X"0000" & imm);
               ELSE
                  rd_reg2 <= (X"FFFF" & imm);
               END IF;
            END IF;

            IF reg_dst = '1' THEN
               wr_reg <= rd;
            ELSE
               wr_reg <= rt;
            END IF;

            state <= "010"; -- EX

         WHEN "010" => -- EX: Execute ALU
            REPORT "state EXE" SEVERITY note;

            CASE alu_op IS
               WHEN alu_add => alu_res <= std_logic_vector(signed(rd_reg1) + signed(rd_reg2));
               WHEN alu_and => alu_res <= and_algo2(rd_reg1, rd_reg2);
               WHEN alu_or => alu_res <= or_algo2(rd_reg1, rd_reg2);
               WHEN alu_ori => alu_res <= ori_algo2(rd_reg1, rd_reg2(15 DOWNTO 0));
               WHEN alu_sub => alu_res <= sub_algo1(rd_reg1, rd_reg2);
               WHEN alu_slt => alu_res <= slt(rd_reg1, rd_reg2);
               WHEN alu_mflo => alu_res <= register_LO;
               WHEN alu_mfhi => alu_res <= register_HI;
               WHEN alu_lui => alu_res <= rd_reg2(15 DOWNTO 0) & X"0000";
               WHEN alu_beq => alu_res <= beq_algo(rd_reg1, rd_reg2);
               WHEN alu_bgez => alu_res <= bgez_algo(rd_reg1);
               WHEN alu_mult => alu_long_res <= std_logic_vector(booth_multiply2(signed(rd_reg1),signed(rd_reg2)));
               WHEN alu_div => IF signed(rd_reg2) /= 0 THEN alu_long_res <= div_long(rd_reg1, rd_reg2); END IF;
               WHEN alu_nop => alu_res <= undefined;
               WHEN OTHERS => alu_res <= dontcare;
            END CASE;    

            CASE alu_op IS
               WHEN alu_add => overflow_flag <= (rd_reg1(31) XOR alu_res(31)) AND (rd_reg2(31) XOR alu_res(31));
               WHEN alu_sub => overflow_flag <= (not(rd_reg1(31))AND(rd_reg2(31))AND(alu_res(31))) OR ((rd_reg1(31))AND(not(rd_reg2(31)))AND(not(alu_res(31))));
               WHEN OTHERS => overflow_flag <= '0';
            END CASE;

            state <= "011"; -- MEM

         WHEN "011" => -- MEM: Check if a read or write is needed
            REPORT "state MEM" SEVERITY note;
            IF mem_read = '1' THEN
               a_bus <= alu_res;
               read <= '1';
               state <= "100"; -- MEM_RD
            ELSIF mem_write = '1' THEN
               a_bus <= alu_res;
               d_busout <= register_file(to_integer(unsigned(wr_reg)));
               write <= '1';
               state <= "101"; -- MEM_WR
            ELSE
               state <= "110"; -- WB
            END IF;

         WHEN "100" => -- MEM_RD: Data memory read
            REPORT "state MEM_RD" SEVERITY note;
            IF ready = '1' THEN
               mem_res <= d_busin;
               read <= '0';
               a_bus <= dontcare;
               state <= "110"; -- WB
            ELSE
               state <= "100";
               read <= '1';
            END IF;


         WHEN "101" => -- MEM_WR: Data memory write and set wr_data
            REPORT "state MEM_WR" SEVERITY note;
            IF ready = '1' THEN
               write <= '0';
               a_bus <= dontcare;
               state <= "110"; -- WB
            ELSE
               state <= "101";
               write <= '1';
            END IF;

         WHEN "110" => -- WB: Write back
            REPORT "state WB" SEVERITY note;
            --Check ALU Result
            IF alu_res = undefined THEN
               ASSERT false REPORT "finished calculation" SEVERITY failure;
            ELSIF alu_res = dontcare THEN
               ASSERT false REPORT "illegal instruction" SEVERITY warning;
            END IF;

            IF mem_to_reg = '1' THEN
               wr_data := mem_res;
            ELSE
               wr_data := alu_res;
            END IF;

            IF reg_write = '1' THEN
               IF alu_op = alu_mult OR alu_op = alu_div THEN
                  register_LO <= alu_long_res(31 DOWNTO 0);
                  register_HI <= alu_long_res(63 DOWNTO 32);
               ELSIF signed(wr_reg) /= 0 THEN  
                  register_file(to_integer(unsigned(wr_reg))) <= wr_data;
               END IF;
            END IF;

            --Increase PC
            IF branch = '1' AND wr_data(0) = '1' THEN
               pc <= pc + 4 + to_integer(signed(imm) SLL 2);
            ELSE
               pc <= pc + 4;
            END IF;
            state <= "000"; -- IF1

         WHEN OTHERS => NULL;

      END CASE;
   END PROCESS;

END rtl;
