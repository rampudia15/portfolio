LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;
USE work.parameters.ALL;

ENTITY average IS
    PORT (
        p_pwm       : IN  std_logic;
        t_pwm       : IN  std_logic;
        rst       : IN  std_logic;
        clk       : IN  std_logic;
        p_avg   	  : OUT REAL;
        t_avg   	  : OUT REAL
        );
END average;

ARCHITECTURE behavioral OF average IS
    CONSTANT p_pwm_period     : INTEGER := clk_freq/p_pwm_freq; --Clk ticks in one pwm period
    CONSTANT p_sampling_period : INTEGER := p_pwm_period / samples_per_period; --Clk ticks in one sampling period
    SIGNAL p_shift_reg : integer_array := (OTHERS => 0);
    SIGNAL p_sum : INTEGER RANGE 0 TO samples;
    
    CONSTANT t_pwm_period     : INTEGER := clk_freq/t_pwm_freq; --Clk ticks in one pwm period
    CONSTANT t_sampling_period : INTEGER := t_pwm_period / samples_per_period; --Clk ticks in one sampling period
    SIGNAL t_shift_reg : integer_array := (OTHERS => 0);
    SIGNAL t_sum : INTEGER RANGE 0 TO samples;

BEGIN

--PAN process
    p_average : PROCESS(clk,rst)
    VARIABLE p_tick_counter : INTEGER RANGE 0 TO p_sampling_period := 0;

	BEGIN
        IF rst = '1' THEN
            p_shift_reg <= (OTHERS => 0);
            p_avg <= 0.0;
            p_tick_counter := 0;
        ELSIF rising_edge(clk) THEN
            IF p_tick_counter = p_sampling_period THEN
                --Moving average of the signal
                IF p_pwm = '1' THEN   	
                    p_shift_reg <= p_shift_reg (samples-2 DOWNTO 0) & 1;
                    p_sum <= p_sum + 1 - p_shift_reg(samples-1);
                ELSIF p_pwm = '0' THEN
                    p_shift_reg <= p_shift_reg (samples-2 DOWNTO 0) & 0;
                    p_sum <= p_sum - p_shift_reg(samples-1);
                END IF;
                p_tick_counter := 0;
            ELSE
                p_tick_counter := p_tick_counter +1;
            END IF;
            
            p_avg <= Real(p_sum) / Real(samples);
        END IF;
    END PROCESS;
    
--TILT process
    t_average : PROCESS(clk,rst)
    VARIABLE t_tick_counter : INTEGER RANGE 0 TO t_sampling_period := 0;

	BEGIN
        IF rst = '1' THEN
            t_shift_reg <= (OTHERS => 0);
            t_avg <= 0.0;
            t_tick_counter := 0;
        ELSIF rising_edge(clk) THEN
            IF t_tick_counter = t_sampling_period THEN
                --Moving average of the signal
                IF t_pwm = '1' THEN   	
                    t_shift_reg <= t_shift_reg (samples-2 DOWNTO 0) & 1;
                    t_sum <= t_sum + 1 - t_shift_reg(samples-1);
                ELSIF t_pwm = '0' THEN
                    t_shift_reg <= t_shift_reg (samples-2 DOWNTO 0) & 0;
                    t_sum <= t_sum - t_shift_reg(samples-1);
                END IF;
                t_tick_counter := 0;
            ELSE
                t_tick_counter := t_tick_counter +1;
            END IF;
            
            t_avg <= Real(t_sum) / Real(samples);
        END IF;
    END PROCESS;

END behavioral;







