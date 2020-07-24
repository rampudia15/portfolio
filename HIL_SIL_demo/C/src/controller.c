#include <stdio.h>
#include "controller.h"

CtrlInputs	ctrlInputs;
CtrlOutputs	ctrlOutputs;

CtrlVariables	vCtrlPan;
CtrlRates	rCtrlPan;
CtrlRates	pCtrlPan; // Previous
CtrlVariables	vCtrlTilt;
CtrlRates	rCtrlTilt;
CtrlRates	pCtrlTilt; // Previous

bool		ctrlInit = false;

void		controller_init()
{
 
  ctrlOutputs.out_pan = 0.0;
  ctrlOutputs.out_tilt = 0.0;
  
  vCtrlPan = {0.0};
  vCtrlTilt = {0.0};
  rCtrlPan = {0.0};
  rCtrlTilt = {0.0};
  pCtrlPan = {0.0};
  pCtrlTilt = {0.0};

}

void		controller_copy_previous()
{
  pCtrlPan.error = rCtrlPan.error;
  pCtrlPan.uD = rCtrlPan.uD;
  pCtrlPan.uI = rCtrlPan.uI;

  pCtrlTilt.error = rCtrlTilt.error;
  pCtrlTilt.uD = rCtrlTilt.uD;
  pCtrlTilt.uI = rCtrlTilt.uI;
}

void		controller_set_inputs(double in_pan, double position_pan, double in_tilt, double position_tilt)
{
  ctrlInputs.in_pan = in_pan;
  ctrlInputs.position_pan = position_pan;
  ctrlInputs.in_tilt = in_tilt;
  ctrlInputs.position_tilt = position_tilt;

  if (!ctrlInit)
    {
      ctrlInit = true;
      controller_execute();
    }
}

double		controller_get_out_pan()
{
  return ctrlOutputs.out_pan;
}

double		controller_get_out_tilt()
{
  return ctrlOutputs.out_tilt;
}

void		controller_execute()
{
  /* ---------- Calculate Dynamics ---------- */


  /* ---------- PID 1 : PAN ---------- */

  /* PID1\factor = 1 / (sampletime + PID1\tauD * PID1\beta); */
  vCtrlPan.factor = 1.0 / (STEP_SIZE + PAN_TAU_D * PAN_BETA);

  /* PlusMinus1\plus1 = in; */
  vCtrlPan.plus1 = ctrlInputs.in_pan;

  /* Splitter1\input = position; */
  vCtrlPan.splitter_input = ctrlInputs.position_pan;

  /* corrGain\corr = corrGain\K * Splitter1\input; */
  vCtrlPan.corr = CORR_GAIN_K * vCtrlPan.splitter_input;

  /* PID1\error = PlusMinus1\plus1 - Splitter1\input; */
  rCtrlPan.error = vCtrlPan.plus1 - vCtrlPan.splitter_input;

  printf("Pan error: %.02f\n", rCtrlPan.error);

  /* PID1\uD = PID1\factor * (((PID1\tauD * PID1\uD_previous) * PID1\beta + (PID1\tauD * PID1\kp) * (PID1\error - PID1\error_previous)) + (sampletime * PID1\kp) * PID1\error); */
  rCtrlPan.uD = vCtrlPan.factor * (((PAN_TAU_D * pCtrlPan.uD) * PAN_BETA
				    + (PAN_TAU_D * PAN_KP) * (rCtrlPan.error - pCtrlPan.error))
				   + (STEP_SIZE * PAN_KP) * rCtrlPan.error);

  /* PID1\uI = PID1\uI_previous + (sampletime * PID1\uD) / PID1\tauI; */
  rCtrlPan.uI = pCtrlPan.uI + (STEP_SIZE * rCtrlPan.uD) / PAN_TAU_I;

  /* PID1\output = PID1\uI + PID1\uD; */
  vCtrlPan.output = rCtrlPan.uI + rCtrlPan.uD;

  /* SignalLimiter2\output = (if PID1\output < SignalLimiter2\minimum then SignalLimiter2\minimum else (if PID1\output > SignalLimiter2\maximum then SignalLimiter2\maximum else PID1\output end) end); */
  vCtrlPan.signal_limiter_out = ((vCtrlPan.output < PAN_SIGNAL_MIN)
				 ? PAN_SIGNAL_MIN
				 : ((vCtrlPan.output > PAN_SIGNAL_MAX)
				    ? PAN_SIGNAL_MAX
				    : vCtrlPan.output));

  
  /* ---------- PID 2 : TILT ---------- */
  
  /* PID2\factor = 1 / (sampletime + PID2\tauD * PID2\beta); */
  vCtrlTilt.factor = 1.0 / (STEP_SIZE + TILT_TAU_D * TILT_BETA);
  
  /* corrGain\input = corr; */
  vCtrlTilt.corr = vCtrlPan.corr;

  /* PlusMinus1\plus1 = in; */
  vCtrlTilt.plus1 = ctrlInputs.in_tilt;

  /* PlusMinus1\minus1 = position; */
  vCtrlTilt.minus1 = ctrlInputs.position_tilt;

  /* corrGain\output = corrGain\K * corrGain\input; */
  vCtrlTilt.corr_out = CORR_GAIN_K * vCtrlTilt.corr;

  /* PID2\error = PlusMinus1\plus1 - PlusMinus1\minus1; */
  rCtrlTilt.error = vCtrlTilt.plus1 - vCtrlTilt.minus1;

  printf("Tilt error: %.02f\n", rCtrlTilt.error);

  /* PID2\uD = PID2\factor * (((PID2\tauD * PID2\uD_previous) * PID2\beta + (PID2\tauD * PID2\kp) * (PID2\error - PID2\error_previous)) + (sampletime * PID2\kp) * PID2\error); */
  rCtrlTilt.uD = vCtrlTilt.factor * (((TILT_TAU_D * pCtrlTilt.uD) * TILT_BETA
				      + (TILT_TAU_D * TILT_KP) * (rCtrlTilt.error - pCtrlTilt.error))
				     + (STEP_SIZE * TILT_KP) * rCtrlTilt.error);

  /* PID2\uI = PID2\uI_previous + (sampletime * PID2\uD) / PID2\tauI; */
  rCtrlTilt.uI = pCtrlTilt.uI + (STEP_SIZE * rCtrlTilt.uD) / TILT_TAU_I;

  /* PID2\output = PID2\uI + PID2\uD; */
  vCtrlTilt.output = rCtrlTilt.uI + rCtrlTilt.uD;

  /* PlusMinus2\output = corrGain\output + PID2\output; */
  vCtrlTilt.plus2_out = vCtrlTilt.corr_out + vCtrlTilt.output;

  /* SignalLimiter2\output = (if PlusMinus2\output < SignalLimiter2\minimum then SignalLimiter2\minimum else (if PlusMinus2\output > SignalLimiter2\maximum then SignalLimiter2\maximum else PlusMinus2\output end) end); */
  vCtrlTilt.signal_limiter_out = ((vCtrlTilt.plus2_out < TILT_SIGNAL_MIN)
				  ? TILT_SIGNAL_MIN
				  : ((vCtrlTilt.plus2_out > TILT_SIGNAL_MAX)
				     ? TILT_SIGNAL_MAX
				     : vCtrlTilt.plus2_out));


  
  /* ---------- Outputs ---------- */

  /* out_pan = SignalLimiter2\output; */
  ctrlOutputs.out_pan = vCtrlPan.signal_limiter_out;

  /* out_tilt = SignalLimiter1\output; */
  ctrlOutputs.out_tilt = vCtrlTilt.signal_limiter_out;

}

void		controller_print_inputs()
{
  printf("Controller inputs:  [p_SP, p_pos, t_SP, t_pos]\n");
  printf("                    [%.02f, %.02f, %.02f, %.02f]\n",
	 ctrlInputs.in_pan, ctrlInputs.position_pan,
	 ctrlInputs.in_tilt, ctrlInputs.position_tilt);
}
