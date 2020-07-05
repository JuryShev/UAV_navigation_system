#ifndef RTC_H_
#define RTC_H_
#ifdef __cplusplus
extern "C" {
#endif
#include <Perefer_lib/I2C.h>
#include "stm32f4xx_hal.h"


uint8_t RTC_ConvertFromDec(uint8_t c);
extern volatile float MX,MY,MZ,M_ZZ, MX_C, MY_C, MZ_C;
void set(float quat[4], float *euler);
void Read_Calc_Data(I2C_HandleTypeDef hi, float qua[], double gyro_dat[], float euler[]);//Read&Calculate_data
#ifdef __cplusplus
    }
    #endif
typedef struct vector 
    {
      float x, y, z;
    } vector;

//vector mag_max= {-2047, -2047, -2047}; // maximum magnetometer values, used for calibration
//vector mag_min= {2047, 2047, 2047}; // minimum magnetometer values, used for calibration
#endif /* __RTC_H */
