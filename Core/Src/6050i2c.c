/*
 * 6050i2c.c
 *
 *  Created on: Jun 5, 2022
 *      Author: Home
 */

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <math.h>
#include "_utils.h"
#include "6050i2c.h"


// Make extern all devices
extern UART_HandleTypeDef huart2;
extern	I2C_HandleTypeDef hi2c2;

extern	Gyro5060_InitTypeDef  MPU6050_Data;




uint8_t serial_buffer[32];

int16_t mpu6050_readydata[6];

/*
 *   endstring[0] = 0x0d;
  endstring[1] = 0x0a;
 */

void I2C_WriteBuffer(uint8_t I2C_ADDRESS, uint8_t *aTxBuffer, uint8_t TXBUFFERSIZE) {
     while(HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)I2C_ADDRESS<<1, (uint8_t*)aTxBuffer, (uint16_t)TXBUFFERSIZE, (uint32_t)1000)!= HAL_OK){
         if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_AF){
        	 // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
            // _Error_Handler(__FILE__, aTxBuffer[0]);
         }

     }

       while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY){}
 }

void I2C_ReadBuffer(uint8_t I2C_ADDRESS, uint8_t RegAddr, uint8_t *aRxBuffer, uint8_t RXBUFFERSIZE){

     I2C_WriteBuffer(I2C_ADDRESS, &RegAddr, 1);

     while(HAL_I2C_Master_Receive(&hi2c2, (uint16_t)I2C_ADDRESS<<1, aRxBuffer, (uint16_t)RXBUFFERSIZE, (uint32_t)1000) != HAL_OK){
         if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_AF){

        	// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
            // _Error_Handler(__FILE__, __LINE__);
         }
     }

     while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY){}
 }


 void MPU6050_Init(void){

     uint8_t buffer[7];

     // power on
     buffer[0] = MPU6050_RA_PWR_MGMT_1;
     buffer[1] = 0x00;
     I2C_WriteBuffer(MPU6050_ADDRESS_AD0_LOW,buffer,2);

     // gyro config
     buffer[0] = MPU6050_RA_GYRO_CONFIG;
     buffer[1] = 0x8; /* normal value for 500°/sec  */
   //  buffer[1] = 0x0; /// normal value for 250° /sec
     I2C_WriteBuffer(MPU6050_ADDRESS_AD0_LOW,buffer,2);

     // config for ±8g
     buffer[0] = MPU6050_RA_ACCEL_CONFIG;
     buffer[1] = 0x10;
     I2C_WriteBuffer(MPU6050_ADDRESS_AD0_LOW,buffer,2);

     /// FIFO and interrupts
//     buffer[0] = MPU6050_RA_INT_PIN_CFG; //x37
//     buffer[1] = 0x00;
//     I2C_WriteBuffer(MPU6050_ADDRESS_AD0_LOW,buffer,2);
//
//     buffer[0] = MPU6050_RA_INT_ENABLE; //x38
//     buffer[1] = 0x01;
//     I2C_WriteBuffer(MPU6050_ADDRESS_AD0_LOW,buffer,2);
     /// end of FIFO and interrupts

     // my settings dlpf - filter

     buffer[0] = MPU6050_RA_CONFIG; //x1a register 26 at page 13 of manual
     buffer[1] = MPU6050_DLPF_BW_188; // в 6050i2c.h - there are some mosed of DLPF
     I2C_WriteBuffer(MPU6050_ADDRESS_AD0_LOW,buffer,2);




     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
     MPU6050_Data.isinterrupt=1;
     MPU6050_Data.fGX_Cal=0;
     MPU6050_Data.fGY_Cal=0;
     MPU6050_Data.fGZ_Cal=0;

     MPU6050_Data.aRoll=0;
     MPU6050_Data.aPitch=0;
     MPU6050_Data.aEileron_left=0;
     MPU6050_Data.aEileron_right=0;

 }


 void MPU6050_GetAllData(int16_t *Data){

    uint8_t accelbuffer[14];

    // Read 14 bytes for all data
    I2C_ReadBuffer(MPU6050_ADDRESS_AD0_LOW,MPU6050_RA_ACCEL_XOUT_H,accelbuffer,14);

    /* Registers 59 to 64 – Accelerometer Measurements */
    for (int i = 0; i< 3; i++)
        Data[i] = ((int16_t) ((uint16_t) accelbuffer[2 * i] << 8) + accelbuffer[2 * i + 1]);

    /* Registers 65 and 66 – Temperature Measurement */


    /* Registers 67 to 72 – Gyroscope Measurements */
    for (int i = 4; i < 7; i++)
        Data[i - 1] = ((int16_t) ((uint16_t) accelbuffer[2 * i] << 8) + accelbuffer[2 * i + 1]);

  }







void HAL_SYSTICK_Callback(void){
 	 static int uiTicksCNT = 0;
 	 static float devider=65.5*50;

 	if(MPU6050_Data.isinitialized && ++uiTicksCNT >= 20) { // 50 times per second
 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

 		MPU6050_GetAllData(mpu6050_readydata);

 		float Roll = mpu6050_readydata[4]- MPU6050_Data.fGY_Cal;
 		Roll = Roll/devider;

 		Roll+=MPU6050_Data.aRoll;

 		//if(fabsf(Roll) > 0.01)

 		float Pitch=(mpu6050_readydata[3] - MPU6050_Data.fGX_Cal);
 		Pitch=Pitch/devider;
 		Pitch+=MPU6050_Data.aPitch;



 		float Yaw = (mpu6050_readydata[5] - MPU6050_Data.fGZ_Cal)/devider;
 		Yaw+=MPU6050_Data.aYaw;


 		// Measure Z movement if it is
 		if(Yaw > 0.01){       // Compare with delta
 		        float _Y = sin(Yaw * 3.1415/180);
 		        Pitch += Roll  * _Y;
 		        Roll -= Pitch * _Y;
 		}

 		// pitch from Acc
 		float pitch4macc = atan2(mpu6050_readydata[1] ,( sqrt(sqr(mpu6050_readydata[0]) + sqr(mpu6050_readydata[2]) ) ) )* R2DEG;
 		// roll from Acc
 		float roll4macc = atan2 (mpu6050_readydata[0] ,( sqrt ( sqr(mpu6050_readydata[1]) + sqr(mpu6050_readydata[2]) ) ) )* R2DEG;

 		MPU6050_Data.aPitch = Pitch * (1-MPU6050_KOEF_COMPL_P) + pitch4macc * MPU6050_KOEF_COMPL_P;
 		MPU6050_Data.aRoll = Roll * (1-MPU6050_KOEF_COMPL_R) + roll4macc * MPU6050_KOEF_COMPL_R;

 		MPU6050_Data.aPitch=Pitch;
 		MPU6050_Data.aRoll=Roll;

 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
 		SendDataSP ();

 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

 	    uiTicksCNT = 0;
 	}





  }

  void MPU6050_Calibrate(void){
 	 HAL_Delay(400);
    int16_t mpu6050data[6];
    uint16_t iNumCM = 1000;

    for (int i = 0; i < iNumCM ; i ++){
      MPU6050_GetAllData(mpu6050data);
      MPU6050_Data.fGX_Cal += mpu6050data[3];
      MPU6050_Data.fGY_Cal += mpu6050data[4];
      MPU6050_Data.fGZ_Cal += mpu6050data[5];

      HAL_Delay(3); // 3 sec for calibration
    }
    MPU6050_Data.fGX_Cal /= iNumCM;
    MPU6050_Data.fGY_Cal /= iNumCM;
    MPU6050_Data.fGZ_Cal /= iNumCM;

  //  IntToSerialPortLN("СAL=", MPU6050_Data.fGX_Cal);

    MPU6050_Data.isinitialized = 1;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(600);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_Delay(600);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    MPU6050_Data.isinterrupt=1;

    MPU6050_Data.int_roll=0;
  }

  void SendDataSP (void) {



	  uint8_t	shift=0;
  	  shift=FloatoSTR(serial_buffer, -1*MPU6050_Data.aRoll, 0 );
  	  serial_buffer[shift]=0x2c; /// 44 in dec
  	  serial_buffer[shift+1]=0x20; /// space
  	  shift=FloatoSTR(serial_buffer, -1*MPU6050_Data.aPitch, shift+2 );


  	  serial_buffer[shift]=0x2c; /// 44 in dec
  	  serial_buffer[shift+1]=0x20; /// space
  	  shift=FloatoSTR(serial_buffer, MPU6050_Data.aEileron_left, shift+2 );

  	  serial_buffer[shift]=0x2c; /// 44 in dec
  	  serial_buffer[shift+1]=0x20; /// space
  	  shift=FloatoSTR(serial_buffer, MPU6050_Data.aEileron_right, shift+2 );

  	 serial_buffer[shift]=0x2c; /// 44 in dec
  	 serial_buffer[shift+1]=0x20; /// space
  	 shift=FloatoSTR(serial_buffer, MPU6050_Data.aYaw, shift+2 );



  	  // End of string - new string
  	  serial_buffer[shift]=0x0d;
  	  serial_buffer[shift+1]=0x0a;



  	HAL_UART_Transmit(&huart2, serial_buffer, shift+2, 49);


  }


