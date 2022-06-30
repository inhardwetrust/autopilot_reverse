/*
 * myservo.c
 *
 *  Created on: May 28, 2022
 *      Author: Home
 */


#include "stm32f1xx_hal.h"
#include <myservo.h>

static TIM_HandleTypeDef  *timer_ptr;
static int16_t max_angle;



 void MyServo_Init(TIM_HandleTypeDef* TIMERx) {

	timer_ptr = TIMERx;


		__HAL_TIM_SET_COMPARE(TIMERx, TIM_CHANNEL_1 , MS_middlepoint+MS_corr1);
		HAL_TIM_PWM_Start(TIMERx, TIM_CHANNEL_1);

		__HAL_TIM_SET_COMPARE(TIMERx, TIM_CHANNEL_2 , MS_middlepoint+MS_corr2);
				HAL_TIM_PWM_Start(TIMERx, TIM_CHANNEL_2);

				MyServo_Rotate(0, 1);
				MyServo_Rotate(0, 2);






 }


 //  void MyServo_Rotate(TIM_HandleTypeDef* TIMERx, int16_t angle) {
 void MyServo_Rotate(int16_t angle, uint8_t channel) {

	 max_angle=angle;
	 if (angle>MS_maxangle) angle=MS_maxangle;
	 if (angle<(MS_maxangle*(-1))) angle=(-1)*MS_maxangle;

	 if (channel==1) __HAL_TIM_SET_COMPARE(timer_ptr, TIM_CHANNEL_1, angle+MS_middlepoint+MS_corr1);
	 if (channel==2) __HAL_TIM_SET_COMPARE(timer_ptr, TIM_CHANNEL_2, angle+MS_middlepoint+MS_corr2);

}

 int16_t MS_PwmtoAngle (int16_t in_angle ) {

	 in_angle=(in_angle/10)-150;

	 return in_angle;
 }
