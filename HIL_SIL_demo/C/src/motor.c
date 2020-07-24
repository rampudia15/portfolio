#include <stdio.h>
#include <math.h>
#include "motor.h"

MotorInputs	motorInputs;
MotorOutputs	motorOutputs;

MotorVariables	vMotorPan;
MotorRates	rMotorPan;
MotorStates	sMotorPan;
MotorVariables	vMotorTilt;
MotorRates	rMotorTilt;
MotorStates	sMotorTilt;

bool		motorInit = false;

void		motor_init()
{
  motorOutputs.out_pan = 0.0;
  motorOutputs.velocity_pan = 0.0;
  motorOutputs.out_tilt = 0.0;
  motorOutputs.velocity_tilt = 0.0;

  vMotorPan = {0.0};
  rMotorPan = {0.0};
  sMotorPan = {0.0};
  vMotorTilt = {0.0};
  rMotorTilt = {0.0};
  sMotorTilt = {0.0};
}

void		motor_euler_step()
{
  sMotorPan.jmot_state = sMotorPan.jmot_state + rMotorPan.jmot_pe * MOTOR_STEP_SIZE;
  sMotorPan.q = sMotorPan.q + rMotorPan.p1_omega * MOTOR_STEP_SIZE;

  sMotorTilt.jmot_state = sMotorTilt.jmot_state + rMotorTilt.jmot_pe * MOTOR_STEP_SIZE;
  sMotorTilt.q = sMotorTilt.q + rMotorTilt.p1_omega * MOTOR_STEP_SIZE;
}

void		motor_set_inputs(double in_pan, double in_tilt)
{
  motorInputs.in_pan = in_pan;
  motorInputs.in_tilt = in_tilt;

  if (!motorInit)
    {
      motorInit = true;
      motor_execute();
    }

}

double		motor_get_out_pan()
{
  return motorOutputs.out_pan;
}

double		motor_get_velocity_pan()
{
  return motorOutputs.velocity_pan;
}

double		motor_get_out_tilt()
{
  return motorOutputs.out_tilt;
}

double		motor_get_velocity_tilt()
{
  return motorOutputs.velocity_tilt;
}

void		motor_execute()
{
  
  /* ----------- PAN ------------ */
  /* Jmot\p.f = Jmot\state / Jmot\i; */
  vMotorPan.jmot_pf = sMotorPan.jmot_state / PAN_JMOT_I;

  /* MSf\flow = Current_Control; */
  vMotorPan.msf_flow = motorInputs.in_pan;

  /* OneJunction3\p3.T = velocity.T; */
  vMotorPan.junction_p3t = vMotorPan.velocity_t;

  /* Motor\p1.e = Motor\r * Jmot\p.f; */
  vMotorPan.motor_p1e = PAN_MOTOR_R * vMotorPan.jmot_pf;

  /* Motor\p2.e = Motor\r * MSf\flow; */
  vMotorPan.motor_p2e = PAN_MOTOR_R * vMotorPan.msf_flow;

  /* Dmot\p.e = Dmot\r * Jmot\p.f; */
  vMotorPan.dmot_pe = PAN_DMOT_R * vMotorPan.jmot_pf;

  /* Gear\p2.f = Gear\r * Jmot\p.f; */
  vMotorPan.gear_p2f = PAN_GEAR_R * vMotorPan.jmot_pf;

  /* phi\p1.omega = Belt\r * Gear\p2.f; */
  rMotorPan.p1_omega = PAN_BELT_R * vMotorPan.gear_p2f;

  /* Dcam\p.e = Dcam\r * phi\p1.omega; */
  vMotorPan.dcam_pe = PAN_DCAM_R * rMotorPan.p1_omega;

  /* Jcam\p.T_in = ((Belt\r * (Gear\r * (((Motor\p2.e - Dmot\p.e) - Gear\r * (Belt\r * (Dcam\p.e + OneJunction3\p3.T))) / Jmot\i))) * Jcam\i) / (1.0 + (Belt\r * (Gear\r * ((Gear\r * Belt\r) / Jmot\i))) * Jcam\i); */		   
  vMotorPan.jcam_pt_in = ((PAN_BELT_R * (PAN_GEAR_R * (((vMotorPan.motor_p2e - vMotorPan.dmot_pe) - PAN_GEAR_R * (PAN_BELT_R * (vMotorPan.dcam_pe + vMotorPan.junction_p3t))) / PAN_JMOT_I))) * PAN_JCAM_I) / (1.0 + (PAN_BELT_R * (PAN_GEAR_R * ((PAN_GEAR_R * PAN_BELT_R) / PAN_JMOT_I))) * PAN_JCAM_I);

  /* Jcam\state = phi\p1.omega * Jcam\i; */
  vMotorPan.jcam_state = rMotorPan.p1_omega * PAN_JCAM_I;

  /* OneJunction3\p4.T = (Dcam\p.e + OneJunction3\p3.T) + Jcam\p.T_in; */
  vMotorPan.junction_p4t = (vMotorPan.dcam_pe + vMotorPan.junction_p3t) + vMotorPan.jcam_pt_in;

  /* Belt\p1.e = Belt\r * OneJunction3\p4.T; */
  vMotorPan.belt_p1e = PAN_BELT_R * vMotorPan.junction_p4t;

  /* Gear\p1.e = Gear\r * Belt\p1.e; */
  vMotorPan.gear_p1e = PAN_GEAR_R * vMotorPan.belt_p1e;

  /* Jmot\p.e = (Motor\p2.e - Dmot\p.e) - Gear\p1.e; */
  rMotorPan.jmot_pe = (vMotorPan.motor_p2e - vMotorPan.dmot_pe) - vMotorPan.gear_p1e;

  
  /* ----------- TILT ------------ */

  /* Jmot\p.f = Jmot\state / Jmot\i; */
  vMotorTilt.jmot_pf = sMotorTilt.jmot_state / TILT_JMOT_I;

  /* MSf\flow = Current_Control; */
  vMotorTilt.msf_flow = motorInputs.in_tilt;

  /* OneJunction3\p3.T = velocity.T; */
  vMotorTilt.junction_p3t = vMotorTilt.velocity_t;

  /* Motor\p1.e = Motor\r * Jmot\p.f; */
  vMotorTilt.motor_p1e = TILT_MOTOR_R * vMotorTilt.jmot_pf;

  /* Motor\p2.e = Motor\r * MSf\flow; */
  vMotorTilt.motor_p2e = TILT_MOTOR_R * vMotorTilt.msf_flow;

  /* Dmot\p.e = Dmot\r * Jmot\p.f; */
  vMotorTilt.dmot_pe = TILT_DMOT_R * vMotorTilt.jmot_pf;

  /* Gear\p2.f = Gear\r * Jmot\p.f; */
  vMotorTilt.gear_p2f = TILT_GEAR_R * vMotorTilt.jmot_pf;

  /* Theta\p1.omega = Belt\r * Gear\p2.f; */
  rMotorTilt.p1_omega = TILT_BELT_R * vMotorTilt.gear_p2f;

  /* Dcam\p.e = Dcam\r * Theta\p1.omega; */
  vMotorTilt.dcam_pe = TILT_DCAM_R * rMotorTilt.p1_omega;

  /* Jcam\p.T_in = ((Belt\r * (Gear\r * (((Motor\p2.e - Dmot\p.e) - Gear\r * (Belt\r * (Dcam\p.e + OneJunction3\p3.T))) / Jmot\i))) * Jcam\i) / (1.0 + (Belt\r * (Gear\r * ((Gear\r * Belt\r) / Jmot\i))) * Jcam\i); */
  vMotorTilt.jcam_pt_in = ((TILT_BELT_R * (TILT_GEAR_R * (((vMotorTilt.motor_p2e - vMotorTilt.dmot_pe) - TILT_GEAR_R * (TILT_BELT_R * (vMotorTilt.dcam_pe + vMotorTilt.junction_p3t))) / TILT_JMOT_I))) * TILT_JCAM_I) / (1.0 + (TILT_BELT_R * (TILT_GEAR_R * ((TILT_GEAR_R * TILT_BELT_R) / TILT_JMOT_I))) * TILT_JCAM_I);

  /* Jcam\state = Theta\p1.omega * Jcam\i; */
  vMotorTilt.jcam_state = rMotorTilt.p1_omega * TILT_JCAM_I;

  /* OneJunction3\p4.T = (Dcam\p.e + OneJunction3\p3.T) + Jcam\p.T_in; */
  vMotorTilt.junction_p4t = (vMotorTilt.dcam_pe + vMotorTilt.junction_p3t) + vMotorTilt.jcam_pt_in;

  /* Belt\p1.e = Belt\r * OneJunction3\p4.T; */
  vMotorTilt.belt_p1e = TILT_BELT_R * vMotorTilt.junction_p4t;

  /* Gear\p1.e = Gear\r * Belt\p1.e; */
  vMotorTilt.gear_p1e = TILT_GEAR_R * vMotorTilt.belt_p1e;

  /* Jmot\p.e = (Motor\p2.e - Dmot\p.e) - Gear\p1.e; */
  rMotorTilt.jmot_pe = (vMotorTilt.motor_p2e - vMotorTilt.dmot_pe) - vMotorTilt.gear_p1e;



  /* ----------- OUTPUTS ------------ */

  motorOutputs.velocity_pan = rMotorPan.p1_omega;
  motorOutputs.velocity_tilt = rMotorTilt.p1_omega;

  motorOutputs.out_pan = sMotorPan.q + rMotorPan.p1_omega * MOTOR_STEP_SIZE;
  motorOutputs.out_tilt = sMotorTilt.q + rMotorTilt.p1_omega * MOTOR_STEP_SIZE;

  motorOutputs.out_pan = roundf(motorOutputs.out_pan * 100) / 100;
  motorOutputs.out_tilt = roundf(motorOutputs.out_tilt * 100) / 100;
}

void		motor_print_inputs()
{
  printf("Motor inputs:       [p_Volt, t_Volt ]\n");
  printf("Motor inputs:       [%0.2f  , %0.2f ]\n",
	 motorInputs.in_pan, motorInputs.in_tilt);
}
