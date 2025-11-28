/*
 * asia.h
 *
 *  Created on: Nov 22, 2025
 *      Author: Joanna Mazur
 */

#ifndef INC_CZUJNIK_H_
#define INC_CZUJNIK_H_


#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_tim.h"
#include <stdio.h>

void Delay_us(uint16_t time);
void read_HCSR04(void);


extern uint32_t distance;

#endif /* INC_CZUJNIK_H_ */
