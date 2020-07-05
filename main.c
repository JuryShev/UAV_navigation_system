/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include <string.h>
#include <strings.h>
#include "stm32f4xx_hal.h"


#include "defines.h"
#include "tm_stm32_hd44780.h"
#include "tm_stm32_usart.h"
#include "tm_stm32_gps.h"

#include "fix16.h"
#include "fixarray.h"
#include "fixquat.h"
#include "fixstring.h"

#include <Perefer_lib/I2C.h>
#include <Perefer_lib/LSM303DLHC.h>
#include <Perefer_lib/RTC.h>
#include <Data_Processing_lib/NAVgo_earthrate.h>
#include <Data_Processing_lib/Kalman_extended.h>

//#include "unittests.h"



/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
char str[256];
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */


float timer;// variable_timer


//____Peripher__________________________________________________________________//

/*_____GPS_Init_________________*/
TM_GPS_Result_t result, current;
TM_GPS_Data_t GPS_Data;
TM_GPS_Custom_t *GPRMC, *GPGLL_lat, *GPGLL_lon, *GPGSA, *GPGGA_ind, *GPGGA_sat, *GPGGA_alt;
TM_GPS_Distance_t GPS_Distance;
//_____________________________________________________________________________//

int main(void)
{


  /* --------------------------MCU Configuration--------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();


  /* Configure the system clock */
  SystemClock_Config();

  timer=HAL_GetTick();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

/*-------------------------------------------------------------------------------------*/

/*_________________GPS_Configuration___________________________________________________*/

  TM_GPS_Init(&GPS_Data, 9600);
  TM_USART_Init ( huart2.Instance ,   TM_USART_PinsPack_1 ,   huart2.Init.BaudRate ) ;
  char GPG_St[]="&";

  	/* $GPRMC statement, term number 7 = Speed over ground in knots */
  sprintf( GPG_St, "$GPRMC");
  GPRMC = TM_GPS_AddCustom(&GPS_Data, GPG_St, 7);//Speed over ground in knots

  	/* $GPGLL statement, term number 1 = Current latitude, 3=Current longitude */
  sprintf( GPG_St, "$GPGLL");
  GPGLL_lat = TM_GPS_AddCustom(&GPS_Data, GPG_St, 1);//Current latitude
  GPGLL_lon = TM_GPS_AddCustom(&GPS_Data, GPG_St, 3);//Current longitude

  	/* $GPGGA statement, term number 9 = MSL Altitude1 */
  sprintf( GPG_St, "$GPGGA");
  GPGGA_alt = TM_GPS_AddCustom(&GPS_Data, GPG_St, 9);//MSL Altitude1

  GPGGA_ind = TM_GPS_AddCustom(&GPS_Data, GPG_St, 6);// GPS quality indicator
  GPGGA_sat = TM_GPS_AddCustom(&GPS_Data, GPG_St, 7);// Number of satellites in use [not those in view]

  /* $GPGSA statement, term number 1 = M = Manual, forced to operate in 2D or 3D A=Automatic, 3D/2D
  sprintf( GPG_St, "$GPGSA");
  GPGSA = TM_GPS_AddCustom(&GPS_Data, GPG_St, 1);
   Add here more custom tags you want */
/*_____________________________________________________________________________________*/

/*______________Variables_for_GPS&INS_integrate________________________________________*/

  	double gyro_dat[3]={0, 0, 0};// gyro_data
    double RM_RN[2]={0,0};       // array for calculates meridian and normal radii of curvature_
    //double omega_en_N[3]={0,0,0};//compensation transport rate
    double omega_ie_N[3]={0,0,0};//compensation rate of the Earth

    double upd[8];//Velocity vector
    double corrected_wb;//correct giro_drif;
    double lat=0;//latitude (radians).
    double lat_g=0;
    double lon=0;//longitude (radians).
    double lon_g=0;
    double h=0;//attitude
    double h_g=0;
    double pos[3]={lat,lon,h};
    double g[3]={0,0,0};//gravity

    float euler[3]={0,0,0};
    float qua[4]={0,0,0,0};
    fix16_t Vn, Ve;// lat-latitude(radian),Vn-velociti nouth(m/s), Ve-velositi East(m/s),velositi up(m/s)

  	mf16 DCMbn = {3, 3, 0,
              {{fix16_from_dbl(0), fix16_from_dbl(0), fix16_from_dbl(0)},
               {fix16_from_dbl(0), fix16_from_dbl(0), fix16_from_dbl(0)},
               {fix16_from_dbl(0), fix16_from_dbl(0), fix16_from_dbl(0)}}};/*matrix rotation*/

  	 mf16 DCMbn_N = {3, 3, 0,
  	              {{fix16_from_dbl(0), fix16_from_dbl(0), fix16_from_dbl(0)},
  	               {fix16_from_dbl(0), fix16_from_dbl(0), fix16_from_dbl(0)},
  	               {fix16_from_dbl(0), fix16_from_dbl(0), fix16_from_dbl(0)}}};/*matrix rotation updated*/

    qf16 qua16 = {fix16_from_dbl(0), fix16_from_dbl(0), fix16_from_dbl(0), fix16_from_dbl(0)};

  	mf16  omega_ie_N2= {3, 1, 0,
              {{fix16_from_int(0)},
               {fix16_from_int(0)},
               {fix16_from_int(0)}}};

    mf16  fb_corrected= {3, 1, 0,
              {{fix16_from_dbl(3.5790)},
               {fix16_from_dbl(-1.3989)},
               {fix16_from_dbl(10.1909)}}};//corrected acc

    mf16  vel_e= {3, 1, 0,
              {{fix16_from_dbl(0)},//0.145
              {fix16_from_dbl(0)},
              {fix16_from_dbl(0)}}};//Velocity vector

    mf16  vel_n= {3, 1, 0,
              {{fix16_from_dbl(0)},
              {fix16_from_dbl(0)},
              {fix16_from_dbl(0)}}};//Velocity vector (vel_update)

    mf16  z= {3, 2, 0,
              {{fix16_from_dbl(0), fix16_from_dbl(0)},
               {fix16_from_dbl(0),fix16_from_dbl(0)},
               {fix16_from_dbl(0),fix16_from_dbl(0)}}};// for function_inovation
  	mf16  fn;
  	uint8_t size;
  	//TM_USART_Puts(USART2, "Init value-Ok \r\n");

  						/*---------------------------Offset-MPU6050-----------------------------------*/

  		int16_t Acc_Gyro_Offs[6];
  		Acc_Gyro_Offs[0]=124;	/*setXGyroOffset*/	Acc_Gyro_Offs[1]=41;  /*setYGyroOffset*/	Acc_Gyro_Offs[2]=39;  /*setZGyroOffset*/

  		Acc_Gyro_Offs[3]=-604; /*setXAccelOffset*/	Acc_Gyro_Offs[4]=382; /*setYAccelOffset*/	Acc_Gyro_Offs[5]=970; /*setZAccelOffset*/

  						/*----------------------------------------------------------------------------*/

  	// _________Disable_sleep_mode_MPU650_____________________
  	aTxBuffer[0]=0x6B;// Adress registr
  	aTxBuffer[1]=0x00;// Komand
  	I2C_WriteBuffer(hi2c1,(uint8_t)0xD0 ,2);
  	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}
  	//__________________________________________________________
  	//TM_USART_Puts(USART2, "Disable_sleep_mode_MPU650-Ok\r\n");

  	// _________Disable_sleep_mode_LSM303DLHC___________________
  	aTxBuffer[0]=0x2;// Adress registr
  	aTxBuffer[1]=0;// Komand
  	I2C_WriteBuffer(hi2c1,(unsigned char)MAG_ADDRESS_WRITE ,2);
  	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}

  	aTxBuffer[0]=0x1;// Adress registr
  	aTxBuffer[1]=0x20;// Komand
  	I2C_WriteBuffer(hi2c1,(unsigned char)MAG_ADDRESS_WRITE ,2);
  	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}

  	aTxBuffer[0]=CRA_REG_M;// Adress registr
  	aTxBuffer[1]=LSM303DLHC_ODR_220_HZ;// Komand
  	I2C_WriteBuffer(hi2c1,(unsigned char)MAG_ADDRESS_WRITE ,2);
  	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}
   //__________________________________________________________
  	//TM_USART_Puts(USART2, "Disable_sleep_mode_LSM303DLHC-Ok\r\n");

   // _______________Offset_MPU650_______________________
   aTxBuffer[0]=0x13;// Adress registr
   aTxBuffer[1]=(Acc_Gyro_Offs[0]>>8)&0xff;// Komand
   aTxBuffer[2]=Acc_Gyro_Offs[0]&0xff;
   I2C_WriteBuffer(hi2c1,(uint16_t)0xD0 ,3);
   while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}

   aTxBuffer[0]=0x15;// Adress registr
   aTxBuffer[1]=(Acc_Gyro_Offs[1]>>8)&0xff;// Komand
   aTxBuffer[2]=Acc_Gyro_Offs[1]&0xff;
   I2C_WriteBuffer(hi2c1,(uint16_t)0xD0 ,3);
   while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}

   aTxBuffer[0]=0x17;// Adress registr
   aTxBuffer[1]=(Acc_Gyro_Offs[2]>>8)&0xff;// Komand
   aTxBuffer[2]=Acc_Gyro_Offs[2]&0xff;
   I2C_WriteBuffer(hi2c1,(uint16_t)0xD0 ,3);
   while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}

   aTxBuffer[0]=0x06;// Adress registr
   aTxBuffer[1]=(Acc_Gyro_Offs[3]>>8)&0xff;// Komand
   aTxBuffer[2]=Acc_Gyro_Offs[3]&0xff;
   I2C_WriteBuffer(hi2c1,(uint16_t)0xD0 ,3);
   while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}

   aTxBuffer[0]=0x08;// Adress registr
   aTxBuffer[1]=(Acc_Gyro_Offs[4]>>8)&0xff;// Komand
   aTxBuffer[2]=Acc_Gyro_Offs[4]&0xff;
   I2C_WriteBuffer(hi2c1,(uint16_t)0xD0 ,3);
   while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}

   aTxBuffer[0]=0x0A;// Adress registr
   aTxBuffer[1]=(Acc_Gyro_Offs[5]>>8)&0xff;// Komand
   aTxBuffer[2]=Acc_Gyro_Offs[5]&0xff;
   I2C_WriteBuffer(hi2c1,(uint16_t)0xD0 ,3);
   while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}
  //____________________________________________________________
   //TM_USART_Puts(USART2, "Offset_MPU650-Ok\r\n");
   //TM_USART_Puts(USART2, "Start_GPs\r\n");

  while (1)
  {


	  timer=HAL_GetTick();
	  Read_Calc_Data(hi2c1, qua, gyro_dat, euler);//Read&Calculate_data
	  size=sprintf((char *)str,"q0=%f	q1=%f	q2=%f, q3=%f	\n\r", gyro_dat[0], gyro_dat[1], gyro_dat[2], qua[3]);
	  TM_USART_Send(USART2, (uint8_t*) str, size);


	  result = TM_GPS_Update(&GPS_Data);
	  //size=sprintf((char *)str,"satelite=%s\n\r", GPGGA_sat->Value);
	  //TM_USART_Send(USART2, (uint8_t*) str, size);

	  if (result == TM_GPS_Result_NewData)
	  	{
		  current = TM_GPS_Result_NewData;

	  			uint8_t satel;
	  			double speed_all;
	  			double del_h=0;

	  			uint8_t ind;
	  			uint8_t simb_pos;

	  			TM_USART_Puts(USART2, "New received data have valid GPS signal\n\r");
	  			TM_USART_Puts(USART2, "---------------------------------------\n\r");
	  			satel=atoi(GPGGA_sat->Value);
	  			ind=atoi(GPGGA_ind->Value);
	  			size=sprintf((char *)str,"satelite=%d\n\r", satel);
	  			TM_USART_Send(USART2, (uint8_t*) str, size);

	  			while(satel>=5)
	  			{
	  				result = TM_GPS_Update(&GPS_Data);
	  				Read_Calc_Data(hi2c1,qua, gyro_dat, euler);//Read&Calculate_data

	  				if (result == TM_GPS_Result_NewData)
	  				{
	  					TM_USART_Puts(USART2, "New received data have valid GPS signal\n");
	  					TM_USART_Puts(USART2, "---------------------------------------\n");

	  					h=atof(GPGGA_alt->Value)-649.35;
	  					del_h=h;
	  					speed_all=atof(GPRMC->Value)*0.44704;// узнать скорость по трем осям

	  					size=sprintf((char *)str,"satelite=%d\n", satel);
	  					TM_USART_Send(USART2, (uint8_t*) str, size);
	  					TM_USART_Puts(USART2, "		");
	  					size=sprintf((char *)str,"alt=%f\n", h);
	  					TM_USART_Send(USART2, (uint8_t*) str, size);
	  				    TM_USART_Puts(USART2, "		");
	  				    size=sprintf((char *)str,"speed=%f\n", speed_all);
	  					TM_USART_Send(USART2, (uint8_t*) str, size);
	  					TM_USART_Puts(USART2, "		");
	  					//current = TM_GPS_Result_NewData; //зажечь светодиод

	  					size=sprintf((char *)str,"%s\n", GPGLL_lat->Value);
	  					simb_pos=(uint8_t*) str[4];

	  					if (simb_pos==46 && size>8)
	  					   {

	  						 int DD = atoi(str)/100;
	  						 float SS=atof(str);
	  						 SS=SS-DD*100;
	  						 lat=DD + SS/60;
	  						 size=sprintf((char *)str,"%f", lat);
	  						 TM_USART_Send(USART2, (uint8_t*) str, size);
	  						 TM_USART_Puts(USART2, "		");

	  					   }

	  					size=sprintf((char *)str,"%s\n", GPGLL_lon->Value);
	  					simb_pos=(uint8_t*) str[5];
	  					if (simb_pos==46 && size>8)
	  						{
	  						  int DD = atoi(str)/100;
	  						  float SS=atof(str);
	  						  SS=SS-DD*100;
	  						  lon=DD + SS/60;
	  						  size=sprintf((char *)str,"%f", lon);
	  						  TM_USART_Send(USART2, (uint8_t*) str, size);
	  						}

	  					earthrate(lat,omega_ie_N);
	  					radius(lat,RM_RN);
	  					TM_USART_Puts(USART2, "\r\n");




	  				}

	  				satel=atoi(GPGGA_sat->Value);
	  				ind=atoi(GPGGA_ind->Value);
	  				timer=(float)HAL_GetTick()/1000;
	  		    }




	  		//printf("New received data have valid GPS signal\n");____
	  		//printf("---------------------------------------\n");_____

	  		/* We have all data from GPS_Data structure valid, you can do here what you want */
	  		/* We will in this example show only custom data to user */

	  }
	  //HAL_UART_Transmit(&huart2, str, 24, 0xFFFF);		//Передача данных
	  HAL_Delay(10);

	  timer=(float)HAL_GetTick()/1000;
	  //size=sprintf((char *)str,"\n\r t=%f",timer);
	  //TM_USART_Send(USART2, (uint8_t*) str, size);



  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
