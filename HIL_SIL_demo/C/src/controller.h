#ifndef CONTROLLER_H_
#define CONTROLLER_H_


#define CORR_GAIN_K	0.0
#define PAN_KP		2.6
#define	PAN_TAU_D	0.05
#define	PAN_BETA	0.17
#define	PAN_TAU_I	9.0
#define	TILT_KP		1.6
#define	TILT_TAU_D	0.05
#define	TILT_BETA	0.001
#define	TILT_TAU_I	10.5
#define PAN_SIGNAL_MIN	-0.99
#define PAN_SIGNAL_MAX	0.99
#define TILT_SIGNAL_MIN	-0.99
#define TILT_SIGNAL_MAX	0.99

#define STEP_SIZE	0.01

typedef struct _CtrlInputs
{
  double		in_pan;
  double		in_tilt;
  double		position_pan;
  double		position_tilt;

} CtrlInputs;

typedef struct _CtrlOutputs
{
  double		out_pan;
  double		out_tilt;
  
} CtrlOutputs;

typedef struct _CtrlVariables
{
  double		corr;
  double		corr_out;
  double		output;
  double		factor;
  double		plus1;
  double		minus1;
  double		plus2_out;
  double		signal_limiter_out;
  double		splitter_input;
  
} CtrlVariables;

typedef struct _CtrlRates
{
  double		error;
  double		uD;
  double		uI;
  
} CtrlRates;


void		controller_init();
void		controller_copy_previous();
void		controller_set_inputs(double in_pan, double position_pan, double in_tilt, double position_tilt);
double		controller_get_out_pan();
double		controller_get_out_tilt();
void		controller_execute();
void		controller_print_inputs();

#endif
