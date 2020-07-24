#ifndef MOTOR_H_
#define MOTOR_H_


#define PAN_BELT_R	5.0
#define PAN_DCAM_R	0.00985
#define PAN_DMOT_R	1.7e-4
#define PAN_GEAR_R	4.0
#define PAN_JCAM_I	0.00108
#define PAN_JMOT_I	2.63e-6
#define PAN_MOTOR_R	0.0394

#define TILT_BELT_R	5.0
#define TILT_DCAM_R	1.35e-5
#define TILT_DMOT_R	1.77e-6
#define TILT_GEAR_R	4.0
#define TILT_JCAM_I	1.0e-4
#define TILT_JMOT_I	2.63e-6
#define TILT_MOTOR_R	0.0394

#define MOTOR_STEP_SIZE	0.01

typedef struct	_MotorInputs
{
  double	in_pan;
  double	in_tilt;

} MotorInputs;


typedef struct	_MotorOutputs
{
  double	out_pan;
  double	velocity_pan;
  double	out_tilt;
  double	velocity_tilt;

} MotorOutputs;


typedef struct	_MotorVariables
{
  double	belt_p1e;
  double	dcam_pe;
  double	dmot_pe;
  double	gear_p1e;
  double	gear_p2f;
  double	jmot_pf;
  double	motor_p1e;
  double	motor_p2e;
  double	msf_flow;
  double	junction_p3t;
  double	junction_p4t;
  double	velocity_t;
  double	jcam_state;
  double	jcam_pt_in;

} MotorVariables;

typedef struct	_MotorStates
{
  double	jmot_state;
  double	q;
  
} MotorStates;

typedef struct	_MotorRates
{
  double	jmot_pe;
  double	p1_omega;
  
} MotorRates;


void		motor_init();
void		motor_euler_step();
void		motor_set_inputs(double in_pan, double in_tilt);
double		motor_get_out_pan();
double		motor_get_velocity_pan();
double		motor_get_out_tilt();
double		motor_get_velocity_tilt();
void		motor_execute();
void		motor_print_inputs();


#endif
