LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;
USE work.parameters.ALL;

ENTITY testbench_demo IS
END testbench_demo;

ARCHITECTURE structure OF testbench_demo IS
  COMPONENT testset_demo
    PORT (p_pid_out   : OUT REAL RANGE -1.0 TO 1.0;
          t_pid_out   : OUT REAL RANGE -1.0 TO 1.0;
          p_enable    : OUT std_logic;
          t_enable    : OUT std_logic;
          rst       : OUT std_logic;
          clk       : OUT std_logic);
  END COMPONENT;
  COMPONENT pid_to_pwm
  PORT (p_pid : IN REAL RANGE -1.0 TO 1.0;
        t_pid : IN REAL RANGE -1.0 TO 1.0;
        rst       : IN std_logic;
        clk       : IN std_logic;
        p_duty : OUT INTEGER RANGE 0 TO 2**p_resolution-1;
        t_duty : OUT INTEGER RANGE 0 TO 2**t_resolution-1;
        p_direction : OUT std_logic;
        t_direction : OUT std_logic);
  END COMPONENT;
  COMPONENT pwm
  PORT (p_duty      : IN INTEGER RANGE 0 TO 2**p_resolution-1; --Sets speed
        t_duty      : IN INTEGER RANGE 0 TO 2**t_resolution-1; --Sets speed
        p_enable    : IN std_logic; --Enables change of duty
        t_enable    : IN std_logic; --Enables change of duty
        rst       : IN std_logic;
        clk       : IN std_logic;
        p_pwm   	  : OUT std_logic;
        t_pwm   	  : OUT std_logic);
  END COMPONENT;
  COMPONENT average
  PORT (p_pwm       : IN  std_logic;
        t_pwm       : IN  std_logic;
        rst       : IN  std_logic;
        clk       : IN  std_logic;
        p_avg   	  : OUT REAL;
        t_avg   	  : OUT REAL
        );
  END COMPONENT;
  COMPONENT plant
  PORT (rst             : IN  std_logic;
        clk             : IN  std_logic;
        p_V       	    : IN  REAL;  --Average voltage from the PWM signal
        t_V       	    : IN  REAL;  --Average voltage from the PWM signal
        p_direction       : IN std_logic;
        t_direction       : IN std_logic;
        p_omega	        : OUT REAL RANGE -1.0*p_max_speed TO p_max_speed := 0.0; -- Angular speed [rad/s]
        t_omega	        : OUT REAL RANGE -1.0*t_max_speed TO t_max_speed := 0.0 -- Angular speed [rad/s]
        );
  END COMPONENT;
  COMPONENT pulse_gen
    PORT (
        p_omega    : IN  REAL RANGE -1.0*p_max_speed TO p_max_speed;  --Simulated rad/s of the motor
        t_omega    : IN  REAL RANGE -1.0*t_max_speed TO t_max_speed;  --Simulated rad/s of the motor
        rst             : IN std_logic;
        clk             : IN std_logic;
        p_a   	        : OUT std_logic;
        p_b             : OUT std_logic;
        t_a   	        : OUT std_logic;
        t_b             : OUT std_logic);
  END COMPONENT;
COMPONENT encoder
  PORT (p_a         : IN  std_logic;
        p_b         : IN std_logic;
        t_a         : IN  std_logic;
        t_b         : IN std_logic;
        rst       : IN std_logic;
        clk       : IN std_logic;
        p_position  : BUFFER INTEGER RANGE 0 TO p_pulses_per_rev;
        p_direction : OUT std_logic;
        t_position  : BUFFER INTEGER RANGE 0 TO t_pulses_per_rev;
        t_direction : OUT std_logic);
END COMPONENT;
COMPONENT pos_to_rad
  PORT (p_position_count : IN INTEGER RANGE 0 TO p_pulses_per_rev;
        t_position_count : IN INTEGER RANGE 0 TO t_pulses_per_rev;
        rst       : IN std_logic;
        clk       : IN std_logic;
        p_position_rad : OUT REAL;
        t_position_rad : OUT REAL);
END COMPONENT;

-- local connections
SIGNAL l_rst     : std_logic;
SIGNAL l_clk     : std_logic;

SIGNAL l_p_pid     : REAL;
SIGNAL l_p_duty    : integer RANGE 0 to 2**p_resolution-1;
SIGNAL l_p_enable  : std_logic;
SIGNAL l_p_pwm     : std_logic;
SIGNAL l_p_avg	 : REAL;
SIGNAL l_p_omega	 : REAL;
SIGNAL l_p_a	: std_logic;
SIGNAL l_p_b	: std_logic;
SIGNAL l_p_position : INTEGER;
SIGNAL l_p_dir_in : std_logic;
SIGNAL l_p_dir_out : std_logic;
SIGNAL l_p_position_rad : REAL;

SIGNAL l_t_pid     : REAL;
SIGNAL l_t_duty    : integer RANGE 0 to 2**t_resolution-1;
SIGNAL l_t_enable  : std_logic;
SIGNAL l_t_pwm     : std_logic;
SIGNAL l_t_avg	 : REAL;
SIGNAL l_t_omega	 : REAL;
SIGNAL l_t_a	: std_logic;
SIGNAL l_t_b	: std_logic;
SIGNAL l_t_position : INTEGER;
SIGNAL l_t_dir_in : std_logic;
SIGNAL l_t_dir_out : std_logic;
SIGNAL l_t_position_rad : REAL;

BEGIN
 ts_demo : testset_demo
      PORT MAP (
      p_pid_out => l_p_pid,
      t_pid_out => l_t_pid,
      p_enable => l_p_enable,
      t_enable => l_t_enable,
      rst => l_rst, 
      clk => l_clk);
pid : pid_to_pwm
  PORT MAP(
        p_pid => l_p_pid,
        t_pid => l_t_pid,
        rst => l_rst,
        clk => l_clk,
        p_duty => l_p_duty,
        t_duty => l_t_duty,
        p_direction => l_p_dir_in,
        t_direction => l_t_dir_in);
 pwm_u : pwm
      PORT MAP (
      p_duty => l_p_duty,
      t_duty => l_t_duty,
      p_enable => l_p_enable,
      t_enable => l_t_enable,
      rst => l_rst,
      clk => l_clk,
      p_pwm => l_p_pwm,
      t_pwm => l_t_pwm);
 avg : average
      PORT MAP (
        p_pwm => l_p_pwm,
        t_pwm => l_t_pwm,         
        rst => l_rst,
        clk => l_clk,
        p_avg => l_p_avg,
        t_avg => l_t_avg);
 plt : plant
      PORT MAP ( 
        rst => l_rst,
        clk => l_clk,
        p_V => l_p_avg,
        t_V => l_t_avg,
        p_direction => l_p_dir_in,
        t_direction => l_t_dir_in,
        p_omega => l_p_omega,
        t_omega => l_t_omega);
 gen : pulse_gen
    PORT MAP(
        p_omega => l_p_omega,
        t_omega => l_t_omega,
        rst => l_rst,             
        clk => l_clk,        
        p_a => l_p_a,
        p_b => l_p_b,
        t_a => l_t_a,
        t_b => l_t_b);
enc_u : encoder
     PORT MAP (
     p_a => l_p_a,
     p_b => l_p_b,
     t_a => l_t_a,
     t_b => l_t_b,
     rst => l_rst,
     clk => l_clk,
     p_position => l_p_position,
     p_direction => l_p_dir_out,
     t_position => l_t_position,
     t_direction => l_t_dir_out);
rad : pos_to_rad
     PORT MAP (
       p_position_count => l_p_position,
       t_position_count => l_t_position,
       rst => l_rst,
       clk => l_clk,
       p_position_rad => l_p_position_rad,
       t_position_rad => l_t_position_rad);
 
END structure;

