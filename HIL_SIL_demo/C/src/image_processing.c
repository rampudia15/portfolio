#include <opencv2/opencv.hpp>
#include "image_processing.h"
#include "math.h"


using namespace cv;


int		run_image_processing(unsigned char *map_data, MotorAngles *angles)
{
  Mat frame(Size(D_WIDTH, D_HEIGHT), CV_8UC3, map_data, Mat::AUTO_STEP);
  Mat hsvFrame;

  // RGB to HSV color
  cvtColor(frame, hsvFrame, COLOR_RGB2HSV);

  // Apply Gaussian blur
  blur(hsvFrame, hsvFrame, Size(1, 1));

  //Threshold
  Mat mask;
  inRange(hsvFrame, Scalar(H_G_LOW,S_G_LOW,V_G_LOW),
	  Scalar(H_G_HIGH,S_G_HIGH,V_G_HIGH), mask);

  //Calculate X,Y center
  Moments mu = moments(mask, true);
  Point centerMask;
  centerMask.x = mu.m10 / mu.m00;
  centerMask.y = mu.m01 / mu.m00;

  circle(mask, centerMask, 5, Scalar(0,0,255));

  if (centerMask.x != -2147483648 && centerMask.y != -2147483648)
    {
      printf("----------------- Image processing -----------------\n");
      printf("Center:     [%d , %d]\n", centerMask.x, centerMask.y);

      // Retrieve pan and tilt angles from center of screen
      angles->pan = (centerMask.x - D_WIDTH/2) * (H_FOV / D_WIDTH) * (M_PI / 180.0);
      angles->tilt = (centerMask.y - D_HEIGHT/2) * (V_FOV / D_HEIGHT) * (M_PI / 180.0);
      
      //Truncate to two decimals
      angles->pan = (roundf(angles->pan * 100) / 100);// * 10;
      angles->tilt = (roundf(angles->tilt * 100) / 100);// * 10;
      
      //Round to nearest 0.05
      angles->pan = (roundf(angles->pan * 20) / 20);// * 10;
      angles->tilt = (roundf(angles->tilt * 20) / 20);// * 10;

      printf("Pan angle:  %.02f\nTilt angle: %.02f\n", angles->pan, angles->tilt);
      return 1;
    }

  return 0;
}
