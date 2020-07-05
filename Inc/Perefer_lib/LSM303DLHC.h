#ifndef LSM303DLHC
#define LSM303DLHC

#include "stm32f4xx_hal.h"
#include "main.h"
#include "RTC.h"

#define MAG_ADDRESS_WRITE  (0x3C)
#define MAG_ADDRESS_READ   (0x3D)
#define LSM303_MR_REG_M    (0x02)
#define CRB_REG_M 0x01
#define CRA_REG_M 0x00

#define LSM303DLHC_ODR_0_75_HZ              ((uint8_t) 0x00)  /*!< Output Data Rate = 0.75 Hz */
#define LSM303DLHC_ODR_1_5_HZ               ((uint8_t) 0x04)  /*!< Output Data Rate = 1.5 Hz */
#define LSM303DLHC_ODR_3_0_HZ               ((uint8_t) 0x08)  /*!< Output Data Rate = 3 Hz */
#define LSM303DLHC_ODR_7_5_HZ               ((uint8_t) 0x0C)  /*!< Output Data Rate = 7.5 Hz */
#define LSM303DLHC_ODR_15_HZ                ((uint8_t) 0x10)  /*!< Output Data Rate = 15 Hz */
#define LSM303DLHC_ODR_30_HZ                ((uint8_t) 0x14)  /*!< Output Data Rate = 30 Hz */
#define LSM303DLHC_ODR_75_HZ                ((uint8_t) 0x18)  /*!< Output Data Rate = 75 Hz */
#define LSM303DLHC_ODR_220_HZ               ((uint8_t) 0x1C)  /*!< Output Data Rate = 220 Hz */

extern volatile int16_t mag_max_buf[3], mag_min_buf[3];
//Function_Colibration
void magcalMPU9250(float * dest1, float * dest2);
void transformation(int16_t buf_mag[], float calibrated_values[]);


#endif /* LSM303DLHC */

