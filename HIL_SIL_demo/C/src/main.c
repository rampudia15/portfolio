#include <signal.h>
#include "gst_capture.h"
#include "image_processing.h"
#include "controller.h"
#include "motor.h"

#define IO_K	20
#define IO_K2	318.30988618379


GstCustomData	gstData;

double		xTime = 0.0;

/* FILE ptr */
FILE		*csvCtrlPanIn;
FILE		*csvCtrlTiltIn;
FILE		*csvCtrlPanPosition;
FILE		*csvCtrlTiltPosition;
FILE		*csvCtrlPanError;
FILE		*csvCtrlTiltError;
FILE		*csvCtrlPanOut;
FILE		*csvCtrlTiltOut;
FILE		*csvMotPanOut;
FILE		*csvMotTiltOut;
FILE		*csvMotPanVelocity;
FILE		*csvMotTiltVelocity;


void	jiwy_execute(unsigned char *map_data)
{
  MotorAngles angles;

  if (!run_image_processing(map_data, &angles))
    return;
  printf("-------------------- Simulation --------------------\n");
  printf("xTime = %.02f\n", xTime);
  xTime += MOTOR_STEP_SIZE;

  fprintf(csvCtrlPanIn, "%.02f,%.02f\n", xTime, angles.pan);
  fprintf(csvCtrlTiltIn, "%.02f,%.02f\n", xTime, angles.tilt);

  double position_pan = motor_get_out_pan();
  double position_tilt = motor_get_out_tilt();

  fprintf(csvCtrlPanPosition, "%.02f,%.02f\n", xTime, position_pan);
  fprintf(csvCtrlTiltPosition, "%.02f,%.02f\n", xTime, position_tilt);

  fprintf(csvCtrlPanError, "%.02f,%.02f\n", xTime, angles.pan - position_pan);
  fprintf(csvCtrlTiltError, "%.02f,%.02f\n", xTime, angles.tilt - position_tilt);

  controller_set_inputs(angles.pan, position_pan, angles.tilt, position_tilt);
  controller_copy_previous();
  controller_print_inputs();
  controller_execute();
 
  double ctrl_out_pan = controller_get_out_pan();
  double ctrl_out_tilt = controller_get_out_tilt();

  printf("Controller outputs: [p_control, t_control]\n"); 
  printf("                    [%0.2f    , %0.2f    ]\n", ctrl_out_pan, ctrl_out_tilt);

  fprintf(csvCtrlPanOut, "%.02f,%.02f\n", xTime, ctrl_out_pan);
  fprintf(csvCtrlTiltOut, "%.02f,%.02f\n", xTime, ctrl_out_tilt);


  /* MOTOR */
  motor_set_inputs(ctrl_out_pan * IO_K, ctrl_out_tilt * IO_K);
  motor_euler_step();
  motor_print_inputs();
  motor_execute();

  double mot_out_pan = motor_get_out_pan();
  double mot_out_tilt = motor_get_out_tilt();

  double mot_velocity_pan = motor_get_velocity_pan();
  double mot_velocity_tilt = motor_get_velocity_tilt();

  printf("Motor outputs:      [p_pos , p_vel, t_pos, t_vel ]\n");
  printf("                    [%.02f , %.02f, %.02f , %.02f ]\n", 
				mot_out_pan, mot_velocity_pan, 
				mot_out_tilt, mot_velocity_tilt);
  printf("----------------------------------------------------\n");
  
  fprintf(csvMotPanOut, "%.02f,%.02f\n", xTime, mot_out_pan);
  fprintf(csvMotTiltOut, "%.02f,%.02f\n", xTime, mot_out_tilt);

  fprintf(csvMotPanVelocity, "%.02f,%.02f\n", xTime, mot_velocity_pan);
  fprintf(csvMotTiltVelocity, "%.02f,%.02f\n", xTime, mot_velocity_tilt);

}

void	exit_handler(int s)
{
  fclose(csvCtrlPanIn);
  fclose(csvCtrlTiltIn);
  fclose(csvCtrlPanPosition);
  fclose(csvCtrlTiltPosition);
  fclose(csvCtrlPanError);
  fclose(csvCtrlTiltError);
  fclose(csvCtrlPanOut);
  fclose(csvCtrlTiltOut);
  fclose(csvMotPanOut);
  fclose(csvMotTiltOut);
  fclose(csvMotPanVelocity);
  fclose(csvMotTiltVelocity);
  
  g_main_loop_quit(gstData.loop);
}

int	main(int ac, char **av)
{

  /* Check input arguments */
  if (ac != 2)
    {
      g_printerr("Usage: %s <device path>\n", av[0]);
      return -1;
    }

  controller_init();
  motor_init();

  csvCtrlPanIn = fopen("Simulation/ctrlInPAN.csv", "w+");
  csvCtrlTiltIn = fopen("Simulation/ctrlInTILT.csv", "w+");
  csvCtrlPanPosition = fopen("Simulation/ctrlPositionPAN.csv", "w+");
  csvCtrlTiltPosition = fopen("Simulation/ctrlPositionTILT.csv", "w+");
  csvCtrlPanError = fopen("Simulation/ctrlErrorPAN.csv", "w+");
  csvCtrlTiltError = fopen("Simulation/ctrlErrorTILT.csv", "w+");
  csvCtrlPanOut = fopen("Simulation/ctrlOutputPAN.csv", "w+");;
  csvCtrlTiltOut = fopen("Simulation/ctrlOutputTILT.csv", "w+");
  csvMotPanOut = fopen("Simulation/motOutputPAN.csv", "w+");
  csvMotTiltOut = fopen("Simulation/motOutputTILT.csv", "w+");
  csvMotPanVelocity = fopen("Simulation/motVelocityPAN.csv", "w+");
  csvMotTiltVelocity = fopen("Simulation/motVelocityTILT.csv", "w+");


  signal(SIGINT, exit_handler);

 
  gst_init(&ac, &av);

  /* Set up the pipeline */
  if (create_gst_pipeline(&gstData, av[1]) != GST_FLOW_OK)
    return -1;

  /* Run loop */
  run_gst_loop(&gstData, av[1]);

  /* Out of the main loop, clean up */
  clean_gst_pipeline(&gstData);

  return 0;
}
