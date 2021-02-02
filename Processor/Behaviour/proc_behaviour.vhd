LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

LIBRARY work;
USE work.processor_types.ALL;
USE work.tools.ALL;
USE work.mem_procedure.ALL;
USE work.memory_config.ALL;

ARCHITECTURE behaviour OF processor IS
BEGIN
   PROCESS
      VARIABLE pc		: natural := text_base_address;
      VARIABLE rs_int		: integer; -- Integer representation of RS
      VARIABLE rt_int		: integer; -- Integer representation of RT
      VARIABLE rd_int		: integer; -- Integer representation of RD
      VARIABLE mult_value	: bit64 := (OTHERS => '0'); -- Temp variable used for multiplication
      VARIABLE hi_value		: bit32 := (OTHERS => '0'); -- HI register value
      VARIABLE lo_value		: bit32 := (OTHERS => '0'); -- LO register value
      VARIABLE instr		: bit32;
         ALIAS op		: bit6 IS instr(31 DOWNTO 26);
         ALIAS rs		: bit5 IS instr(25 DOWNTO 21);
         ALIAS rt		: bit5 IS instr(20 DOWNTO 16);
         ALIAS rd		: bit5 IS instr(15 DOWNTO 11);
         ALIAS sha		: bit5 IS instr(10 DOWNTO 6);
         ALIAS func		: bit6 IS instr(5 DOWNTO 0);
         ALIAS imm		: bit16 IS instr(15 DOWNTO 0);

      VARIABLE overflow_flag	: std_logic := '0';

      VARIABLE register_file	: registers := (OTHERS => X"00000000");

      VARIABLE tmp		: bit32 := (OTHERS => '0'); -- Use to store the value read from data memory

   BEGIN

      REPORT "Behaviour" SEVERITY note;

      -- Read next instruction
      mem_read(pc, instr, clk, reset, ready, read, write, a_bus, d_busin);

      IF reset /= '1' THEN

	 pc := pc + 4; -- Increment to next instruction

         rs_int := to_integer(unsigned(rs));
         rt_int := to_integer(unsigned(rt));
         rd_int := to_integer(unsigned(rd));

         REPORT "pc is : " & integer'image(pc-text_base_address) & "; instruction is: " & to_hstr(instr) SEVERITY note;
      
         IF instr = nop THEN -- NOP Instruction
            ASSERT false REPORT "finished calculation" SEVERITY failure;
            WAIT;

         ELSIF op = op_rtype THEN -- R-Type Instruction
            REPORT "function is: " & op_to_string(func) SEVERITY note;

            CASE (func) IS
               WHEN func_and | func_or | func_add | func_sub | func_slt => -- Instr Rd, Rs, Rt
                  CASE (func) IS
                     WHEN func_and =>
                        REPORT "Instr: AND" SEVERITY note;
                        REPORT to_hstr(register_file(rs_int)) & " & " & to_hstr(register_file(rt_int));
                        register_file(rd_int) := register_file(rs_int) AND register_file(rt_int);
                     WHEN func_or =>
                        REPORT "Instr: OR" SEVERITY note;
                        REPORT to_hstr(register_file(rs_int)) & " | " & to_hstr(register_file(rt_int));
                        register_file(rd_int) := register_file(rs_int) OR register_file(rt_int);
                     WHEN func_add =>
                        REPORT "Instr: ADD" SEVERITY note;
                        REPORT to_hstr(register_file(rs_int)) & " + " & to_hstr(register_file(rt_int));
                        IF signed(register_file(rs_int)) > 0 AND signed(register_file(rt_int)) > 0
                        AND (signed(register_file(rs_int)) + signed(register_file(rt_int))) < 0 THEN
                           ASSERT false REPORT "overflow" SEVERITY warning;
                           overflow_flag := '1';
                        ELSIF signed(register_file(rs_int)) < 0 AND signed(register_file(rt_int)) < 0
                        AND (signed(register_file(rs_int)) + signed(register_file(rt_int))) > 0 THEN
                           ASSERT false REPORT "underflow" SEVERITY warning;
                           overflow_flag := '1';
                        END IF;
                        register_file(rd_int) := std_logic_vector(signed(register_file(rs_int)) + signed(register_file(rt_int)));
                     WHEN func_sub =>
                        REPORT "Instr: SUB" SEVERITY note;
                        REPORT to_hstr(register_file(rs_int)) & " - " & to_hstr(register_file(rt_int));
                        IF signed(register_file(rs_int)) > 0 AND signed(register_file(rt_int)) < 0
                        AND (signed(register_file(rs_int)) - signed(register_file(rt_int))) < 0 THEN
                           ASSERT false REPORT "overflow" SEVERITY warning;
                           overflow_flag := '1';
                        ELSIF signed(register_file(rs_int)) = 0 AND signed(register_file(rt_int)) = -2147483648 THEN
                           ASSERT false REPORT "overflow" SEVERITY warning;
                           overflow_flag := '1';
                        ELSIF signed(register_file(rs_int)) < 0 AND signed(register_file(rt_int)) > 0
                        AND (signed(register_file(rs_int)) - signed(register_file(rt_int))) > 0 THEN
                           ASSERT false REPORT "underflow" SEVERITY warning;
                           overflow_flag := '1';
                        END IF;
                        register_file(rd_int) := std_logic_vector(signed(register_file(rs_int)) - signed(register_file(rt_int)));
                     WHEN func_slt =>
                        REPORT "Instr: SLT" SEVERITY note;
                        REPORT to_hstr(register_file(rs_int)) & " < " & to_hstr(register_file(rt_int));
                        IF (signed(register_file(rs_int)) < signed(register_file(rt_int))) THEN
                           register_file(rd_int) := (0 => '1', OTHERS => '0');
                        ELSE
                           register_file(rd_int) := (OTHERS => '0');
                        END IF;
                     WHEN OTHERS => NULL;
                  END CASE;
                  REPORT "RD: " & integer'image(rd_int) & "; RS: " & integer'image(rs_int) & "; RT: " & integer'image(rt_int) SEVERITY note;
		  REPORT "R[" & integer'image(rd_int) & "] = " & to_hstr(register_file(rd_int));

               WHEN func_div | func_mult => -- Instr Rs, Rt
                  CASE (func) IS
                     WHEN func_div =>
                        REPORT "Instr: DIV" SEVERITY note;
                        REPORT to_hstr(register_file(rs_int)) & " / " & to_hstr(register_file(rt_int));
                        IF signed(register_file(rt_int)) /= 0 THEN
   	                   lo_value := std_logic_vector(signed(register_file(rs_int)) / signed(register_file(rt_int)));
	                   hi_value := std_logic_vector(signed(register_file(rs_int)) REM signed(register_file(rt_int)));
                        END IF;
                     WHEN func_mult =>
                        REPORT "Instr: MULT" SEVERITY note;
                        REPORT to_hstr(register_file(rs_int)) & " * " & to_hstr(register_file(rt_int));
                        mult_value := std_logic_vector(signed(register_file(rs_int)) * signed(register_file(rt_int)));
                        hi_value := mult_value(63 DOWNTO 32);
                        lo_value := mult_value(31 DOWNTO 0);
                     WHEN OTHERS => NULL;
                  END CASE;
                  REPORT "RS: " & integer'image(rs_int) & "; RT: " & integer'image(rt_int) SEVERITY note;
                  REPORT "LO: " & to_hstr(lo_value) & "; HI: " & to_hstr(hi_value) SEVERITY note;


               WHEN func_mflo | func_mfhi => -- Instr Rd
                  CASE (func) IS
                     WHEN func_mflo =>
                        REPORT "Instr: MFLO" SEVERITY note;
                        register_file(rd_int) := lo_value;
                     WHEN func_mfhi =>
                        REPORT "Instr: MFHI" SEVERITY note;
                        register_file(rd_int) := hi_value;
                     WHEN OTHERS => NULL;
                  END CASE;
                  REPORT "RD: " & integer'image(rd_int) SEVERITY note;
		  REPORT "R[" & integer'image(rd_int) & "] = " & to_hstr(register_file(rd_int));

               WHEN OTHERS => ASSERT false REPORT "illegal instruction" SEVERITY warning;
            END CASE;

         ELSE -- I-Type Instruction
            REPORT "op code is: " & op_to_string(op) SEVERITY note;
            CASE (op) IS
               WHEN op_ori | op_addi => -- Instr Rt, Rs, Imm
                  CASE (op) IS
                     WHEN op_ori =>
                        REPORT "Instr: ORI" SEVERITY note;
                        REPORT to_hstr(register_file(rs_int)) & " | " & integer'image(to_integer(signed(imm)));
                        register_file(rt_int) := register_file(rs_int) or (X"0000" & imm);
                     WHEN op_addi =>
                        REPORT "Instr: ADDI" SEVERITY note;
                        REPORT to_hstr(register_file(rs_int)) & " + " & integer'image(to_integer(signed(imm)));
                        IF signed(register_file(rs_int)) > 0 AND signed(imm) > 0
                        AND (signed(register_file(rs_int)) + signed(imm)) < 0 THEN
                           ASSERT false REPORT "overflow" SEVERITY warning;
                           overflow_flag := '1';
                        ELSIF signed(register_file(rs_int)) < 0 AND signed(imm) < 0
                        AND (signed(register_file(rs_int)) + signed(imm)) > 0 THEN
                           ASSERT false REPORT "underflow" SEVERITY warning;
                           overflow_flag := '1';
                        END IF;
                        register_file(rt_int) := std_logic_vector(signed(register_file(rs_int)) + signed(imm));
                     WHEN OTHERS => NULL;
                  END CASE;
                  REPORT "RT: " & integer'image(rt_int) & "; RS: " & integer'image(rs_int) & "; Imm: " & integer'image(to_integer(signed(imm))) SEVERITY note;
		  REPORT "R[" & integer'image(rt_int) & "] = " & to_hstr(register_file(rt_int));

               WHEN op_lw | op_sw => -- Instr Rt, offset(Rs)
                  CASE (op) IS
                     WHEN op_lw =>
                        REPORT "Instr: LW" SEVERITY note;
                        mem_read((to_integer(signed(register_file(rs_int))+signed(imm))), tmp, clk, reset, ready, read, write, a_bus, d_busin);
                        IF rt_int > 1 THEN
                           register_file(rt_int) := tmp;
                        END IF;
                     WHEN op_sw =>
                        REPORT "Instr: SW" SEVERITY note;
                        mem_write((to_integer(signed(register_file(rs_int))+signed(imm))), register_file(rt_int), clk, reset, ready, read, write, a_bus, d_busout);
                     WHEN OTHERS => NULL;
                  END CASE;
                  REPORT "RT: " & integer'image(rt_int) & "; RS: " & integer'image(rs_int) & "; Imm: " & integer'image(to_integer(signed(imm))) SEVERITY note;
                  REPORT "R[" & integer'image(rs_int) & "] = " & to_hstr(register_file(rs_int));
                  REPORT "R[" & integer'image(rt_int) & "] = " & to_hstr(register_file(rt_int));

               WHEN op_bgez => -- Instr Rs, Label (Imm ?)
                  REPORT "Instr: BGEZ" SEVERITY note;
                  IF signed(register_file(rs_int)) >= 0 THEN
                     pc := pc + to_integer(signed(imm) SLL 2);
                  END IF;
                  REPORT "RS: " & integer'image(rs_int) & "; Imm: " & integer'image(to_integer(signed(imm))) SEVERITY note;

               WHEN op_beq => -- Instr Rs, Rt, Label (Imm ?)
                  REPORT "Instr: BEQ" SEVERITY note;
                  IF signed(register_file(rs_int)) = signed(register_file(rt_int)) THEN
                     pc := pc + to_integer(signed(imm) SLL 2);
                  END IF;
                  REPORT "RS: " & integer'image(rs_int) & "; Rt: " & integer'image(rt_int) & "; Imm: " & integer'image(to_integer(signed(imm))) SEVERITY note;

               WHEN op_lui => -- Instr Rt, Imm
                  REPORT "Instr: LUI" SEVERITY note;
		  register_file(rt_int) := imm & X"0000";
                  REPORT "RT: " & integer'image(rt_int) & "; Imm: " & integer'image(to_integer(signed(imm))) SEVERITY note;
                  REPORT "R[" & integer'image(rt_int) & "] = " & to_hstr(register_file(rt_int));


               WHEN OTHERS => ASSERT false REPORT "illegal instruction" SEVERITY warning;
            END CASE;
         END IF;
      END IF;

   END PROCESS;
END behaviour;