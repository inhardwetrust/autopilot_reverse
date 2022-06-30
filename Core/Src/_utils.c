/*
 * _utils.c
 *
 *  Created on: 15 черв. 2022 р.
 *      Author: Денис
 */

#include "stm32f1xx_hal.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <_utils.h>



uint8_t FloatoSTR(uint8_t *buff, float _float, uint8_t shift  ) {



	int16_t arr_index = shift;

	int16_t _int_value;
	char tempbuf[32];

	uint8_t num;
	_int_value=(int)(_float*10);
	num=sprintf(tempbuf, "%d", _int_value);

	if (abs(_float)>=1) {
		tempbuf[num]=tempbuf[num-1];
		tempbuf[num-1]=0x2e;

	} else {

		tempbuf[num+1]=tempbuf[num-1];
		tempbuf[num]=0x2e;
		tempbuf[num-1]=0x30;
		num++;

	}

	num++;
	tempbuf[num]=0;

	for(int i=0; i<num; i++) {

		buff[arr_index]=tempbuf[i];
		arr_index++;

	}

	return arr_index;

}

