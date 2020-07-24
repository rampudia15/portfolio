#ifndef IMAGE_PROCESSING_H_
#define IMAGE_PROCESSING_H_

#include <math.h>
#include <stdio.h>

#define D_WIDTH		320
#define D_HEIGHT	240

#define M_PI		3.14159265358979323846

#define H_G_LOW		60
#define H_G_HIGH	127
#define S_G_LOW		126
#define S_G_HIGH	255
#define V_G_LOW		73
#define V_G_HIGH	255


#define D_FOV		63 // Diagonal FOV
#define H_FOV		52.23
#define V_FOV		40.37

typedef struct _MotorAngles
{
  double	pan;
  double	tilt;

} MotorAngles;

int		run_image_processing(unsigned char *map, MotorAngles *angles);

#endif
