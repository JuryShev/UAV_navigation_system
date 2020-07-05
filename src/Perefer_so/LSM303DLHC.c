#include "LSM303DLHC.h"


volatile int16_t mag_max_buf[3]={0,0,0}, mag_min_buf[3]={0,0,0};
float uncalibrated_values[3]={0,0,0};


//______________Function_Colibration________________
void magcalMPU9250(float * dest1, float * dest2 )
{
		int16_t mag_bias[3]={0,0,0},mag_scale[3] = {0, 0, 0};
		
		mag_bias[0]  = (mag_max_buf[0] + mag_min_buf[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max_buf[1] + mag_min_buf[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max_buf[2] + mag_min_buf[2])/2;  // get average z mag bias in counts
    
    dest1[0] = (float) mag_bias[0]/1100;  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]/1100;   
    dest1[2] = (float) mag_bias[2]/980;  
       
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max_buf[0] - mag_min_buf[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max_buf[1] - mag_min_buf[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max_buf[2] - mag_min_buf[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0f;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);
	
}	
//___________________________________________________

void transformation(int16_t buf_mag[], float calibrated_values[])
{
  //calibration_matrix[3][3] is the transformation matrix
  //replace M11, M12,..,M33 with your transformation matrix data
	float uncalibrated_values[3]={0,0,0};
	uncalibrated_values[0]=buf_mag[0];
	uncalibrated_values[1]=buf_mag[1];
	uncalibrated_values[2]=buf_mag[2];
	
  double calibration_matrix[3][3] = 
  {
    {1.186, -0.096, -0.066},
    {0.046, 1.331, 0.145},
    {0.093, 0.071, 1.725}  
  };
  //bias[3] is the bias
  //replace Bx, By, Bz with your bias data
  double bias[3] = 
  {
    -39.445,
    -7.62,
    359.683
  };  
  //calculation
  for (int i=0; i<3; ++i) uncalibrated_values[i] = uncalibrated_values[i] - bias[i];
  float result[3] = {0, 0, 0};
  for (int i=0; i<3; ++i)
    for (int j=0; j<3; ++j)
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
  for (int i=0; i<3; ++i) calibrated_values[i] = result[i];
}
