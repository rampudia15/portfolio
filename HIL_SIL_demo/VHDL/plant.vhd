LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;
USE work.parameters.ALL;

ENTITY plant IS
    PORT (rst             : IN  std_logic;
        clk             : IN  std_logic;
        p_V       	    : IN  REAL;  --Average voltage from the PWM signal
        t_V       	    : IN  REAL;  --Average voltage from the PWM signal
        p_direction       : IN std_logic;
        t_direction       : IN std_logic;
        p_omega	        : OUT REAL RANGE -1.0*p_max_speed TO p_max_speed := 0.0; -- Angular speed [rad/s]
        t_omega	        : OUT REAL RANGE -1.0*t_max_speed TO t_max_speed := 0.0 -- Angular speed [rad/s]
        );
END plant;

ARCHITECTURE simulation OF plant IS
    SIGNAL p_old_th :  REAL  := 0.0;
    SIGNAL p_th    :   REAL := 0.0;
    SIGNAL p_old_w :   REAL := 0.0;
    SIGNAL p_w     :   REAL := 0.0;
    SIGNAL p_dw    :   REAL := 0.0;
    SIGNAL p_old_i :   REAL := 0.0;
    SIGNAL p_i     :   REAL := 0.0;
    SIGNAL p_di    :   REAL := 0.0;
    SIGNAL p_V_s   :   REAL;
    
    SIGNAL t_old_th :  REAL  := 0.0;
    SIGNAL t_th    :   REAL := 0.0;
    SIGNAL t_old_w :   REAL := 0.0;
    SIGNAL t_w     :   REAL := 0.0;
    SIGNAL t_dw    :   REAL := 0.0;
    SIGNAL t_old_i :   REAL := 0.0;
    SIGNAL t_i     :   REAL := 0.0;
    SIGNAL t_di    :   REAL := 0.0;
    SIGNAL t_V_s   :   REAL;
    
    CONSTANT dt  : REAL := Real(clk_freq)*dt_period/2.0; --Clk cycles in one pwm period

BEGIN

--PAN process  
    p_plant : PROCESS(clk,rst)

    VARIABLE p_tick_counter : REAL RANGE 0.0 TO dt := 0.0;

	BEGIN
	IF rst = '1' THEN
        p_old_th <= 0.0;
        p_th <= 0.0;
        p_old_w <= 0.0;
        p_w <= 0.0;
        p_dw <= 0.0;
        p_old_i <= 0.0;
        p_i <= 0.0;
        p_di <= 0.0; 
	ELSIF rising_edge(clk) THEN
    
        IF p_direction = '1' THEN 
            p_V_s <= p_V;
        ELSIF p_direction = '0' THEN
            p_V_s <= -1.0*p_V;
        END IF;
         
    
        IF p_tick_counter = dt THEN
        
            p_dw <= (1.0/p_J)*(p_Kt*p_i - p_b*p_w);--dw=-(b/J)*w + Kt*i; 
            p_di <= (1.0/p_L)*(p_V_s*p_K - p_i*p_R - p_Ke*p_w); --di=(1/L)*(V-i*R-Ke*w);
            p_w <= p_old_w + p_dw*dt_period*speedup_sim; --w=w+dw*dt;
            p_th <= p_old_th + p_w*dt_period*speedup_sim; --th=th+w*dt;
            p_i <= p_old_i + p_di*dt_period*speedup_sim; --i=i+di*dt;
        
            p_old_th <= p_th;
            p_old_w <= p_w;
            p_old_i <= p_i;
        
            p_omega <= p_w / p_gearing;

            p_tick_counter := 0.0;
            
		ELSE
			p_tick_counter := p_tick_counter +1.0;
		END IF;
	END IF;
    END PROCESS;
    
--TILT process 
    t_plant : PROCESS(clk,rst)

    VARIABLE t_tick_counter : REAL RANGE 0.0 TO dt := 0.0;

	BEGIN
	IF rst = '1' THEN
        t_old_th <= 0.0;
        t_th <= 0.0;
        t_old_w <= 0.0;
        t_w <= 0.0;
        t_dw <= 0.0;
        t_old_i <= 0.0;
        t_i <= 0.0;
        t_di <= 0.0; 
	ELSIF rising_edge(clk) THEN
    
        IF t_direction = '1' THEN 
            t_V_s <= t_V;
        ELSIF t_direction = '0' THEN
            t_V_s <= -1.0*t_V;
        END IF;
            
        IF t_tick_counter = dt THEN
        
            t_dw <= (1.0/t_J)*(t_Kt*t_i - t_b*t_w);--dw=-(b/J)*w + Kt*i; 
            t_di <= (1.0/t_L)*(t_V_s*t_K - t_i*t_R - t_Ke*t_w); --di=(1/L)*(V-i*R-Ke*w);
            t_w <= t_old_w + t_dw*dt_period*speedup_sim; --w=w+dw*dt;
            t_th <= t_old_th + t_w*dt_period*speedup_sim; --th=th+w*dt;
            t_i <= t_old_i + t_di*dt_period*speedup_sim; --i=i+di*dt;
        
            t_old_th <= t_th;
            t_old_w <= t_w;
            t_old_i <= t_i;
        
            t_omega <= t_w / t_gearing;

            t_tick_counter := 0.0;
            
		ELSE
			t_tick_counter := t_tick_counter +1.0;
		END IF;
	END IF;
    END PROCESS;

END simulation;









