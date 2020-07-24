LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;
USE work.parameters.ALL;

ENTITY pos_to_rad IS
  PORT (p_position_count : IN INTEGER RANGE -1*p_pulses_per_rev*10 TO p_pulses_per_rev*10;
        t_position_count : IN INTEGER RANGE -1*t_pulses_per_rev*10 TO t_pulses_per_rev*10;
        rst       : IN std_logic;
        clk       : IN std_logic;
        p_position_rad : OUT REAL;
        t_position_rad : OUT REAL);
END pos_to_rad;

ARCHITECTURE behavioral OF pos_to_rad IS

BEGIN

   PROCESS(clk, rst)
      BEGIN
      
      IF rst = '1' THEN
         p_position_rad <= 0.0;
         t_position_rad <= 0.0;
      ELSE 
        IF rising_edge(clk) THEN
            p_position_rad <= Real(p_position_count)*2.0*pi/Real(p_pulses_per_rev);
            t_position_rad <= Real(t_position_count)*2.0*pi/Real(t_pulses_per_rev);
        END IF; 
      END IF;
      
   END PROCESS;

END behavioral;






