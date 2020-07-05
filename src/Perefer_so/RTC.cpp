#include "RTC.h"
#include "math.h"
#ifdef __cplusplus
extern "C" {
#endif
#include "LSM303DLHC.h"
#include "MadgwickAHRS.h"
#include "filter_proizv.h"
#ifdef __cplusplus
    }
    #endif
#include "main.h"


uint8_t RTC_ConvertFromDec(uint8_t c)
{
		uint8_t ch=(c>>4)*10+(0x0F&c);
		return ch;
}

void set(float quat[4], float *euler)
{
   // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth.
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    float roll,pitch,yaw,Yaw,Pitch,Roll;
    /*yaw   = atan2(2.0f * (quat[1] * quat[2] + quat[0] * quat[3]), quat[0] * quat[0] + quat[1] * quat[1] - quat[2] * quat[2] - quat[3] * quat[3]);
    pitch = -asin(2.0f * (quat[1] * quat[3] - quat[0] * quat[2]));
    roll  = atan2(2.0f * (quat[0] * quat[1] + quat[2] * quat[3]), quat[0] * quat[0] - quat[1] * quat[1] - quat[2] * quat[2] + quat[3] * quat[3]);
    pitch *= 180.0f / Pi;
    yaw   *= 180.0f / Pi;
    yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
    roll  *= 180.0f / Pi;
    euler[0]=roll;
    euler[1]=pitch;
    euler[1]=yaw;*/

    //Hardware AHRS:
    Yaw   = atan2(2.0f * (quat[0] * quat[1] + quat[3] * quat[2]), quat[3] * quat[3] + quat[0] * quat[0] - quat[1] * quat[1] - quat[2] * quat[2]);
    Pitch = -asin(2.0f * (quat[0] * quat[2] - quat[3] * quat[1]));
    Roll  = atan2(2.0f * (quat[3] * quat[0] + quat[1] * quat[2]), quat[3] * quat[3] - quat[0] * quat[0] - quat[1] * quat[1] + quat[2] * quat[2]);



    Pitch *= 180.0f / Pi;
    Yaw   *= 180.0f / Pi;
    Yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    if(Yaw < 0) Yaw   += 360.0f ; // Ensure yaw stays between 0 and 360
    Roll  *= 180.0f / Pi;
    euler[0]=Roll;
    euler[1]=Pitch;
    euler[2]=Yaw;
}

//___________Read&Calculate_data_________________________


void Read_Calc_Data(I2C_HandleTypeDef hi, float qua[], double gyro_dat[],float euler[])
{
	vector m; vector a;	vector gyro; vector delta; vector del;// вывести при необходимости
	int16_t tempRaw;
	int16_t buf_mag[3]={0,0,0};
	float	calibrated_values[3]={0,0,0};
	char k=0;
	uint8_t size;
	
	/*--------------Init-&-Conf-Parameters-filter-proizv---------------------------*/
	
	/*   	Acc						   Gcc							Mag			   */
		filter aX;					filter gX;					filter mX;
		filter aY;					filter gY;					filter mY;
		filter aZ;					filter gZ;					filter mZ;
	
	aX.setvarVolt(49.67); 		gX.setvarVolt(13.34);		mX.setvarVolt(1.93);
    aX.setProcess(0.8);			gX.setProcess(0.1);			mX.setProcess(0.008);
	aY.setvarVolt(56.03);		gY.setvarVolt(15.21);		mY.setvarVolt(1.99);
	aY.setProcess(0.8);			gY.setProcess(0.1);			mY.setProcess(0.008);
	aZ.setvarVolt(72.18);		gZ.setvarVolt(11.25);		mZ.setvarVolt(1.6);
	aZ.setProcess(0.8);			gZ.setProcess(0.1);			mZ.setProcess(0.008);
	
	/*-----------------------------------------------------------------------------*/
	
	//___Read_Data_MPU650__________________________________
	aTxBuffer[0]=0x3B;
	I2C_WriteBuffer(hi,(uint16_t) 0xD0 ,1);
	while (HAL_I2C_GetState(&hi) != HAL_I2C_STATE_READY)
   {
			
   }
	I2C_ReadBuffer(hi,(uint16_t)0xD0,14);
		
	a.x = (int16_t)((aTxBuffer[0] << 8) | aTxBuffer[1]);
	a.y = (int16_t)((aTxBuffer[2] << 8) | aTxBuffer[3]);
	a.z = (int16_t)((aTxBuffer[4] << 8) | aTxBuffer[5]);  
	tempRaw = (int16_t)((aTxBuffer[6] << 8) | aTxBuffer[7]);  
	gyro.x = (int16_t)((aTxBuffer[8] << 8) | aTxBuffer[9]);
	gyro.y = (int16_t)((aTxBuffer[10] << 8) | aTxBuffer[11]);
	gyro.z = (int16_t)((aTxBuffer[12] << 8) | aTxBuffer[13]);
	memset(aTxBuffer, 0, sizeof(aTxBuffer));
	//_______________________________________________________
	 
	//___Read_Data_LSM303DLHC________________________________
		aTxBuffer[0]=0x3;
		I2C_WriteBuffer(hi,(uint16_t) MAG_ADDRESS_WRITE ,1);
		while (HAL_I2C_GetState(&hi) != HAL_I2C_STATE_READY)
    {
			
    }
		I2C_ReadBuffer(hi,(uint16_t)MAG_ADDRESS_READ,6);
		
		buf_mag[0]  = (int16_t) ((aTxBuffer[0] << 8) | aTxBuffer[1]);
		buf_mag[1]  = (int16_t)((aTxBuffer[2] << 8) | aTxBuffer[3]);
		buf_mag[2]  = (int16_t)((aTxBuffer[4] << 8) | aTxBuffer[5]);
		memset(aTxBuffer, 0, sizeof(aTxBuffer));

		transformation(buf_mag, calibrated_values);

		calibrated_values[0]=mX.filter_value(calibrated_values[0]);
		calibrated_values[1]=mY.filter_value(calibrated_values[1]);
		calibrated_values[2]=mZ.filter_value(calibrated_values[2]);
		
		if(buf_mag[0]!=-4096) m.y=(-calibrated_values[0]); //m.x
		if(buf_mag[1]!=-4096) m.x=calibrated_values[1]; //m.y
		if(buf_mag[2]!=-4096) m.z=calibrated_values[2]; //m.z
	 //___________________________________________________________
		
	//___ Calculate the angles_________________________________
		
		gyro.x=gX.filter_value((float) gyro.x);
		gyro.y=gY.filter_value((float)gyro.y);
		gyro.z=gZ.filter_value((float)gyro.z);
		
		gyro.x = ((float)gyro.x/131.0)*Pi/180;
		gyro.y = ((float)gyro.y/131.0)*Pi/180;
		gyro.z = ((float)gyro.z/131.0)*Pi/180;
		
		gyro_dat[0]=gyro.x;
		gyro_dat[1]=gyro.y;
		gyro_dat[2]=gyro.z;// to exit array
		
		a.x=aX.filter_value((float)a.x);
		a.y=aY.filter_value((float)a.y);
		a.z=aZ.filter_value((float)a.z);
		
		a.x = ((float)a.x/16387.0);
		a.y = ((float)a.y/16387.0);
		a.z = ((float)a.z/16387.0);
		
		
		
		
		


		//MX=gyro.z;
		//MY=calibrated_values[0];
		//MZ=calibrated_values[2];
		
		//del.y=a.y;
		
		MX_C=calibrated_values[0]/1100;
		MY_C=calibrated_values[1]/1100;
		MZ_C=calibrated_values[2]/980;
		
		for(k=0;k<=10;k++)
			{
				MadgwickAHRSupdate(-gyro.x, -gyro.y, -gyro.z, a.x, a.y, a.z, -m.x, m.y, m.z);

			}
		qua[0]=q0;
		qua[1]=q1;
		qua[2]=q2;
		qua[3]=q3;
		set(qua, euler); 
	//___________________________________________________________
}


//_______________________________________________________
volatile float MX = 1.0f, MY = 0.0f, MZ = 0.0f, MX_C=0.0f, MY_C=0.0f, MZ_C=0.0f;	// quaternion of sensor frame relative to auxiliary frame
