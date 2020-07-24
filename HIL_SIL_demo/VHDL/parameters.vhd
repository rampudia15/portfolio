LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

PACKAGE parameters IS

   --Parameters of the PWM Average block
    CONSTANT samples  : INTEGER := 500; --Number of samples used to calculate average of a PWM signal
    CONSTANT samples_per_period  : INTEGER := 100; --Number of samples per pwm period to calculate PWM (used for sampling period)

    --Some types
    TYPE integer_array IS ARRAY (integer range samples-1 DOWNTO 0) OF integer;
    
    --Global clk (freq)
    CONSTANT clk_freq    : INTEGER := 50_000_000; 
    
    --Simulation parameters
    CONSTANT dt_period : REAL := 0.01; --Time step of the simulation [ms]
    CONSTANT speedup_sim : REAL := 1.0; -- Speeds up the simulation times the value of the parameter
    
------------------------------------------------------------- PAN ----------------------------------------------------------------
    --Parameters for the Encoder module
    CONSTANT p_pulses_per_rev   : integer := 2000;
    
    --Parameters for the PWM module
    CONSTANT p_pwm_freq    : INTEGER := 10_000;    --Desired PWM frequency (Hz)
    CONSTANT p_resolution  : INTEGER := 10;
    
    --Parameters for the Encoder Pulse Generator block
    CONSTANT p_max_speed : REAL := 240.0; -- [rad/sec] roughly equal to 2300 rpm
    CONSTANT pi       : REAL := 3.14159;
    
    --Plant parameters
    CONSTANT p_K  : REAL := 0.0394*20; --Power Amplifier
    CONSTANT p_J  : REAL := 0.00108; --Moment of intertia [kg.m^2]
    CONSTANT p_b  : REAL := 0.00985; --Damping coefficient [N.m.s]
    CONSTANT p_Ke : REAL := 0.000173; --Electromotive force [V/rad/sec]
    CONSTANT p_Kt : REAL := 0.000173; --Motor torque constant [N.m/Amp]
    CONSTANT p_R  : REAL := 0.00017; --Electrical resistance [Ohm]
    CONSTANT p_L  : REAL := 0.00000263; --Electrical inductance [H]
    CONSTANT p_gearing : REAL := 4.0*5.0; -- Belt*Gear ratio

------------------------------------------------------------- TILT ----------------------------------------------------------------
        --Parameters for the Encoder module
    CONSTANT t_pulses_per_rev   : integer := 1800;
    
    --Parameters for the PWM module
    CONSTANT t_pwm_freq    : INTEGER := 10_000;    --Desired PWM frequency (Hz)
    CONSTANT t_resolution  : INTEGER := 10;
    
    --Parameters for the Encoder Pulse Generator block
    CONSTANT t_max_speed : REAL := 240.0; -- [rad/sec] roughly equal to 2300 rpm
    
    --Plant parameters
    CONSTANT t_K  : REAL := 0.0394*20; --Power Amplifier 
    CONSTANT t_J  : REAL := 0.0001; --Moment of intertia [kg.m^2]
    CONSTANT t_b  : REAL := 0.0000135; --Damping coefficient [N.m.s]
    CONSTANT t_Ke : REAL := 0.00000208; --Electromotive force [V/rad/sec]
    CONSTANT t_Kt : REAL := 0.00000208; --Motor torque constant [N.m/Amp]
    CONSTANT t_R  : REAL := 0.00000177; --Electrical resistance [Ohm]
    CONSTANT t_L  : REAL := 0.00000263; --Electrical inductance [H]
    CONSTANT t_gearing : REAL := 4.0*5.0; --belt*Gear ratio

END parameters;