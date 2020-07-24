LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;
USE work.parameters.ALL;

ENTITY encoder IS
  PORT (p_a         : IN  std_logic;
        p_b         : IN std_logic;
        t_a         : IN  std_logic;
        t_b         : IN std_logic;
        rst       : IN std_logic;
        clk       : IN std_logic;
        p_position  : BUFFER INTEGER RANGE -1*p_pulses_per_rev*10 TO p_pulses_per_rev*10; --Allows up to 10 turns
        p_direction : OUT std_logic;
        t_position  : BUFFER INTEGER RANGE -1*t_pulses_per_rev*10 TO t_pulses_per_rev*10; --Allows up to 10 turns
        t_direction : OUT std_logic
        --bus_out : OUT signed (15 DOWNTO 0) -- For HW implementation, not necessary for now
        );
END encoder;

ARCHITECTURE rtl OF encoder IS
   SIGNAL p_old_state_temp : std_logic_vector(1 DOWNTO 0);
   SIGNAL t_old_state_temp : std_logic_vector(1 DOWNTO 0);

BEGIN

--PAN process
   p_enc : PROCESS(clk, rst)
      VARIABLE p_state     : std_logic_vector(1 DOWNTO 0);
      VARIABLE p_old_state : std_logic_vector(1 DOWNTO 0);
 
      BEGIN
      
      IF rst = '1' THEN
        p_state := "00";
        p_old_state := "00";
        p_old_state_temp <= "00";
        p_position <= 0;
        p_direction <= '0';
      ELSE 
         IF rising_edge(clk) THEN

            p_state := p_a & p_b;
            p_old_state := p_old_state_temp;
            p_old_state_temp <= p_state;
      
            CASE p_old_state & p_state IS
               --CW motion
               WHEN "0010" | "1011" | "1101" | "0100" =>
                    p_direction <= '0';
                    p_position <= p_position - 1;
               --CCW motion
               WHEN "0001" | "0111" | "1110" | "1000" =>
                    p_direction <= '1';
                    p_position <= p_position + 1;
               WHEN OTHERS =>
            END CASE;
         END IF;
      END IF;
      
      --bus_out <= to_signed(p_position, 16);
   END PROCESS;

--TILT process 
      t_enc : PROCESS(clk, rst)
      VARIABLE t_state     : std_logic_vector(1 DOWNTO 0);
      VARIABLE t_old_state : std_logic_vector(1 DOWNTO 0);
 
      BEGIN
      
      IF rst = '1' THEN
        t_state := "00";
        t_old_state := "00";
        t_old_state_temp <= "00";
        t_position <= 0;
        t_direction <= '0';
      ELSE 
         IF rising_edge(clk) THEN

            t_state := t_a & t_b;
            t_old_state := t_old_state_temp;
            t_old_state_temp <= t_state;
      
            CASE t_old_state & t_state IS
               --CW motion
               WHEN "0010" | "1011" | "1101" | "0100" =>
                  t_direction <= '0';
                    t_position <= t_position - 1;
               --CCW motion
               WHEN "0001" | "0111" | "1110" | "1000" =>
                   t_direction <= '1';
                   t_position <= t_position + 1;
               WHEN OTHERS =>
            END CASE;
         END IF;
      END IF;
   END PROCESS;

END rtl;






