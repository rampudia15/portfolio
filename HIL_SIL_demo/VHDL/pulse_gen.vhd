LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;
USE work.parameters.ALL;

ENTITY pulse_gen IS
    PORT (
        p_omega    : IN  REAL RANGE -1.0*p_max_speed TO p_max_speed;  --Simulated rad/s of the motor
        t_omega    : IN  REAL RANGE -1.0*t_max_speed TO t_max_speed;  --Simulated rad/s of the motor
        rst             : IN std_logic;
        clk             : IN std_logic;
        p_a   	        : OUT std_logic;
        p_b             : OUT std_logic;
        t_a   	        : OUT std_logic;
        t_b             : OUT std_logic);
END pulse_gen;

ARCHITECTURE behavioral OF pulse_gen IS
    SIGNAL p_state    :   std_logic_vector(1 DOWNTO 0) := "00";
    SIGNAL p_pulse_period : INTEGER := 0;
    SIGNAL p_omega_s : REAL := 0.0;
    SIGNAL p_old_omega_s : REAL := 0.0;
    SIGNAL p_direction : std_logic := '1';
    
    SIGNAL t_state    :   std_logic_vector(1 DOWNTO 0) := "00";
    SIGNAL t_pulse_period : INTEGER := 0;
    SIGNAL t_omega_s : REAL := 0.0;
    SIGNAL t_old_omega_s : REAL := 0.0;
    SIGNAL t_direction : std_logic := '1';

BEGIN
--PAN process
    p_pulse_gen : PROCESS(clk,rst)
    VARIABLE p_tick_counter : INTEGER := 0;

	BEGIN
	IF rst = '1' THEN
		p_a <= '0';
		p_b <= '0';
        p_state <= "00";
        p_pulse_period <= 0;
	ELSIF rising_edge(clk) THEN
		--If the clock tick counter matches one pulse_period perform routine       
        IF p_omega <= 0.0 THEN 
            p_omega_s <= -1.0*p_omega;
            p_direction <= '0';
        ELSE
            p_omega_s <= p_omega;
            p_direction <= '1';
        END IF;
        
        --Below a threshold the speed is considered 0 and no pulse period is calculated
        IF p_omega_s >= 0.005 THEN 
            p_pulse_period <= clk_freq/INTEGER(p_omega_s*Real(p_pulses_per_rev)/(2.0*pi)); 
        END IF;
        
		IF p_tick_counter >= p_pulse_period THEN 
            IF p_omega_s <= 0.005 THEN
                p_a <= p_state(0);
                p_b <= p_state(1);
            ELSE   
            --Continute the pattern depending on the current p_state and direction
                CASE p_state IS
                    WHEN "00" =>
                        IF p_direction = '1' THEN
                            p_a <= '0';
                            p_b <= '1';
                            p_state <= "01";
                        ELSIF p_direction = '0' THEN
                            p_a <= '1';
                            p_b <= '0';
                            p_state <= "10";
                        END IF;
                    WHEN "01" =>
                        IF p_direction = '1' THEN
                            p_a <= '1';
                            p_b <= '1';
                            p_state <= "11";
                        ELSIF p_direction = '0' THEN
                            p_a <= '0';
                            p_b <= '0';
                            p_state <= "00";
                        END IF;
                    WHEN "11" =>
                        IF p_direction = '1' THEN
                            p_a <= '1';
                            p_b <= '0';
                            p_state <= "10";
                        ELSIF p_direction = '0' THEN
                            p_a <= '0';
                            p_b <= '1';
                            p_state <= "01";
                        END IF;
                    WHEN "10" =>
                        IF p_direction = '1' THEN
                            p_a <= '0';
                            p_b <= '0';
                            p_state <= "00";
                        ELSIF p_direction = '0' THEN
                            p_a <= '1';
                            p_b <= '1';
                            p_state <= "11";
                        END IF;
                    WHEN OTHERS => 
                END CASE;            
            END IF;
            
            --Reset tick counter
			p_tick_counter := 0;
		ELSE
			p_tick_counter := p_tick_counter +1;
		END IF;
	END IF;
    END PROCESS;
    
--TILT process  
    t_pulse_gen : PROCESS(clk,rst)
    VARIABLE t_tick_counter : INTEGER := 0;

	BEGIN
	IF rst = '1' THEN
		t_a <= '0';
		t_b <= '0';
        t_state <= "00";
        t_pulse_period <= 0;
	ELSIF rising_edge(clk) THEN
    
		--If the clock tick counter matches one pulse_period perform routine       
        IF t_omega <= 0.0 THEN 
            t_omega_s <= -1.0*t_omega;
            t_direction <= '0';
        ELSE
            t_omega_s <= t_omega;
            t_direction <= '1';
        END IF;
        
        IF t_omega_s >= 0.005 THEN
            t_pulse_period <= clk_freq/INTEGER(t_omega_s*Real(t_pulses_per_rev)/(2.0*pi)); 
        END IF;
        
		IF t_tick_counter >= t_pulse_period THEN 
            IF t_omega_s <= 0.005 THEN
                --pulse_period <= 0;
                t_a <= t_state(0);
                t_b <= t_state(1);
            ELSE  
            --Continute the pattern depending on the current p_state and direction
                CASE t_state IS
                    WHEN "00" =>
                        IF t_direction = '1' THEN
                            t_a <= '0';
                            t_b <= '1';
                            t_state <= "01";
                        ELSIF t_direction = '0' THEN
                            t_a <= '1';
                            t_b <= '0';
                            t_state <= "10";
                        END IF;
                    WHEN "01" =>
                        IF t_direction = '1' THEN
                            t_a <= '1';
                            t_b <= '1';
                            t_state <= "11";
                        ELSIF t_direction = '0' THEN
                            t_a <= '0';
                            t_b <= '0';
                            t_state <= "00";
                        END IF;
                    WHEN "11" =>
                        IF t_direction = '1' THEN
                            t_a <= '1';
                            t_b <= '0';
                            t_state <= "10";
                        ELSIF t_direction = '0' THEN
                            t_a <= '0';
                            t_b <= '1';
                            t_state <= "01";
                        END IF;
                    WHEN "10" =>
                        IF t_direction = '1' THEN
                            t_a <= '0';
                            t_b <= '0';
                            t_state <= "00";
                        ELSIF t_direction = '0' THEN
                            t_a <= '1';
                            t_b <= '1';
                            t_state <= "11";
                        END IF;
                    WHEN OTHERS => 
                END CASE;            
            END IF;
            
            --Reset tick counter
			t_tick_counter := 0;
		ELSE
			t_tick_counter := t_tick_counter +1;
		END IF;
	END IF;
    END PROCESS;

END behavioral;








