/*
 * myservo.h
 *
 *  Created on: May 28, 2022
 *      Author: Home
 */

#ifndef SRC_MYSERVO_H_
#define SRC_MYSERVO_H_

#define MS_corr1	0
#define MS_corr2	0
#define MS_middlepoint	150
#define MS_maxangle	30


#include "main.h"
#include "stm32f1xx_hal.h"




void MyServo_Init(TIM_HandleTypeDef* TIMERx);
void MyServo_Rotate( int16_t angle, uint8_t channel);
int16_t MS_PwmtoAngle (int16_t in_angle );




#endif /* SRC_MYSERVO_H_ */



