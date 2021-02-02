LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

LIBRARY work;
USE work.processor_types.ALL;

PACKAGE mem_procedure IS

   PROCEDURE mem_read (addr		: IN natural;
                       result		: OUT bit32;
                       SIGNAL clk	: IN std_ulogic;
                       SIGNAL reset	: IN std_ulogic;
                       SIGNAL ready	: IN std_ulogic;
                       SIGNAL read	: OUT std_ulogic;
                       SIGNAL write	: OUT std_ulogic;
                       SIGNAL a_bus	: OUT bit32;
                       SIGNAL d_busin	: IN bit32);

   PROCEDURE mem_write (addr		: IN natural;
                        data		: IN bit32;
                        SIGNAL clk	: IN std_ulogic;
                        SIGNAL reset	: IN std_ulogic;
                        SIGNAL ready	: IN std_ulogic;
                        SIGNAL read	: OUT std_ulogic;
                        SIGNAL write	: OUT std_ulogic;
                        SIGNAL a_bus	: OUT bit32;
                        SIGNAL d_busout	: OUT bit32);
  
END mem_procedure;

PACKAGE BODY mem_procedure IS

   PROCEDURE mem_read (addr		: IN natural;
                       result		: OUT bit32;
                       SIGNAL clk	: IN std_ulogic;
                       SIGNAL reset	: IN std_ulogic;
                       SIGNAL ready	: IN std_ulogic;
                       SIGNAL read	: OUT std_ulogic;
                       SIGNAL write	: OUT std_ulogic;
                       SIGNAL a_bus	: OUT bit32;
                       SIGNAL d_busin	: IN bit32) IS
   BEGIN
      WAIT UNTIL clk='0';
      IF reset='1' THEN
         RETURN;
      END IF;
      -- put address on output
      a_bus <= std_logic_vector(to_unsigned(addr,32));
      write <='0';
      read<='1';
      LOOP
         IF reset='1' THEN
            RETURN;
         END IF;
         EXIT WHEN ready='1';
         WAIT UNTIL clk='0';
      END LOOP;
      result := d_busin;
      read <='0';
      LOOP
         IF reset='1' THEN
            RETURN;
         END IF;
         EXIT WHEN ready='0';
         WAIT UNTIL clk='0';
      END LOOP; 
      a_bus <= dontcare;
   END mem_read;

   PROCEDURE mem_write (addr		: IN natural;
                        data		: IN bit32;
                        SIGNAL clk	: IN std_ulogic;
                        SIGNAL reset	: IN std_ulogic;
                        SIGNAL ready	: IN std_ulogic;
                        SIGNAL read	: OUT std_ulogic;
                        SIGNAL write	: OUT std_ulogic;
                        SIGNAL a_bus	: OUT bit32;
                        SIGNAL d_busout	: OUT bit32) IS
   BEGIN
      WAIT UNTIL clk = '0';
      IF reset='1' THEN
         RETURN;
      END IF;
      -- put address on output
      a_bus <= std_logic_vector(to_unsigned(addr,32));
      d_busout <= data;
      read <= '0';
      write <= '1';
      LOOP
         IF reset='1' THEN
            RETURN;
         END IF;
         EXIT WHEN ready='1';
         WAIT UNTIL clk='0';
      END LOOP;
      write <='0';
      LOOP
         IF reset='1' THEN
            RETURN;
         END IF;
         EXIT WHEN ready='0';
         WAIT UNTIL clk='0';
      END LOOP; 
      a_bus <= dontcare;
      d_busout <= dontcare;
   END mem_write;

END PACKAGE BODY mem_procedure;