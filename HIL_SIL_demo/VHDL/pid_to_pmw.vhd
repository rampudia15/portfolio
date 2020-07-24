LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;
USE work.parameters.ALL;

ENTITY pid_to_pwm IS
PORT (  p_pid : IN REAL RANGE -1.0 TO 1.0;
        t_pid : IN REAL RANGE -1.0 TO 1.0;
        rst       : IN std_logic;
        clk       : IN std_logic;
        p_duty : OUT INTEGER RANGE 0 TO 2**p_resolution-1;
        t_duty : OUT INTEGER RANGE 0 TO 2**t_resolution-1;
        p_direction : OUT std_logic;
        t_direction : OUT std_logic);
END pid_to_pwm;

ARCHITECTURE behavioral OF pid_to_pwm IS

BEGIN
--PAN process
   p_pid_to_pwm : PROCESS(clk, rst)
      BEGIN   
      IF rst = '1' THEN
        p_duty <= 0;
      ELSE 
        IF rising_edge(clk) THEN          
            IF p_pid < 0.0 THEN
                p_direction <= '0';
                p_duty <= integer(-1.0*p_pid*Real(2**p_resolution-1));
            ELSE
                p_direction <= '1';
                p_duty <= integer(p_pid*Real(2**p_resolution-1));
            END IF;
        END IF; 
      END IF;    
   END PROCESS;
   
 --TILT process  
      t_pid_to_pwm : PROCESS(clk, rst)
      BEGIN   
      IF rst = '1' THEN
        t_duty <= 0;
      ELSE 
        IF rising_edge(clk) THEN          
            IF t_pid < 0.0 THEN
                t_direction <= '0';
                t_duty <= integer(-1.0*t_pid*Real(2**t_resolution-1));
            ELSE
                t_direction <= '1';
                t_duty <= integer(t_pid*Real(2**t_resolution-1));
            END IF;
        END IF; 
      END IF;
      
   END PROCESS;

END behavioral;






