LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;
USE work.parameters.ALL;

ENTITY pwm IS
  PORT (p_duty      : IN INTEGER RANGE 0 TO 2**p_resolution-1;
        t_duty      : IN INTEGER RANGE 0 TO 2**t_resolution-1;
        p_enable    : IN std_logic; --Enables change of duty
        t_enable    : IN std_logic; --Enables change of duty
        rst       : IN std_logic;
        clk       : IN std_logic;
        p_pwm   	  : OUT std_logic;
        t_pwm   	  : OUT std_logic);
END pwm;

ARCHITECTURE rtl OF pwm IS
   CONSTANT p_pwm_period     : INTEGER := clk_freq/p_pwm_freq; --Clk cycles in one pwm period
   CONSTANT p_pwm_period_res : INTEGER := p_pwm_period/2**p_resolution; --Clk cycles in one pwm resolution period
   SIGNAL p_duty_en : INTEGER RANGE 0 TO p_pwm_period;

   CONSTANT t_pwm_period     : INTEGER := clk_freq/t_pwm_freq; --Clk cycles in one pwm period
   CONSTANT t_pwm_period_res : INTEGER := t_pwm_period/2**t_resolution; --Clk cycles in one pwm resolution period
   SIGNAL t_duty_en : INTEGER RANGE 0 TO t_pwm_period;

BEGIN

--PAN process
   p_pwm_set : PROCESS(clk,rst)
    VARIABLE p_tick_counter : INTEGER RANGE 0 TO p_pwm_period;
    VARIABLE p_tick_counter_res : INTEGER RANGE 0 TO p_pwm_period_res;

	BEGIN
	IF rst = '1' THEN
		p_duty_en <= 0;
        p_pwm <= '0';
		p_tick_counter := 0;
        p_tick_counter_res := 0;
	ELSIF rising_edge(clk) THEN
		--If the clock tick counter matches one pwm_period and enable is true, allow new duty
        IF p_tick_counter >= p_pwm_period THEN
            
             IF p_enable = '1' THEN 
                --Convert from duty in % to the pwm_period ticks
				p_duty_en <= (p_pwm_period*p_duty)/(2**p_resolution-1);
            END IF;
            
            p_tick_counter := 0;
            p_tick_counter_res := 0;
            
		ELSIF p_tick_counter_res = p_pwm_period_res THEN
     
            --Set output
            IF p_duty_en = 0 THEN
                p_pwm <= '0';
            ELSIF p_tick_counter <= p_duty_en THEN
                p_pwm <= '1';
            ELSE
                p_pwm <= '0';
	       END IF;    
           
           p_tick_counter := p_tick_counter +1;
           p_tick_counter_res := 0;           
		ELSE
            p_tick_counter_res := p_tick_counter_res +1;
			p_tick_counter := p_tick_counter +1;
		END IF;
    END IF;
   END PROCESS;
  
  
--TILT process  
    t_pwm_set : PROCESS(clk,rst)
    VARIABLE t_tick_counter : INTEGER RANGE 0 TO t_pwm_period;
    VARIABLE t_tick_counter_res : INTEGER RANGE 0 TO t_pwm_period_res;

	BEGIN
	IF rst = '1' THEN
		t_duty_en <= 0;
        t_pwm <= '0';
		t_tick_counter := 0;
        t_tick_counter_res := 0;
	ELSIF rising_edge(clk) THEN
		--If the clock tick counter matches one pwm_period and enable is true, allow new duty
        IF t_tick_counter >= t_pwm_period THEN
            
             IF t_enable = '1' THEN 
                --Convert from duty in % to the pwm_period ticks
				t_duty_en <= (t_pwm_period*t_duty)/(2**t_resolution-1);
            END IF;
            
            t_tick_counter := 0;
            t_tick_counter_res := 0;
            
		ELSIF t_tick_counter_res = t_pwm_period_res THEN
     
            --Set output
            IF t_duty_en = 0 THEN
                t_pwm <= '0';
            ELSIF t_tick_counter <= t_duty_en THEN
                t_pwm <= '1';
            ELSE
                t_pwm <= '0';
	       END IF;    
           
           t_tick_counter := t_tick_counter +1;
           t_tick_counter_res := 0;           
		ELSE
            t_tick_counter_res := t_tick_counter_res +1;
			t_tick_counter := t_tick_counter +1;
		END IF;
    END IF;
   END PROCESS;

END rtl;







