/*
 * asia.c
 *
 *  Created on: Nov 22, 2025
 *      Author: Joanna Mazur
 */
#include "czujnik.h"
#include "tim.h"
#include "gpio.h"

void Delay_us(uint16_t time){
	//ustawia wartość licznika na 0 (licznik ma okres 1us)
	__HAL_TIM_SET_COUNTER(&htim1, 0);

	uint16_t start=__HAL_TIM_GET_COUNTER(&htim1);
	while((uint16_t)(__HAL_TIM_GET_COUNTER(&htim1)-start)<time){;}
}

uint32_t FirstVal; //start impulsu na echo
uint8_t capturestate =0;
uint32_t SecondVal; //koniec impulsu na echo
uint32_t impulse_width;
volatile uint32_t distance=5;
uint32_t new_distance=5;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) //nadpisanie _weak funkcji z "stm32f4xx_hal_tim.c "
{
	if(!capturestate){
		FirstVal = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_3);
		capturestate = 1;
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
		__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC1);
	}
	else{
		SecondVal = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_3);
		capturestate=0;
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
		__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC1);

		if (SecondVal >= FirstVal) //licznik nie został przepełniony
		    impulse_width = SecondVal - FirstVal;
		else //licznik przepełniony
		    impulse_width = (TIM1->ARR - FirstVal) + SecondVal;

		//wzor z dokumentacji
		new_distance = (uint32_t)((float)impulse_width / 2.0 * 0.0343);
		if(new_distance <40) distance=new_distance;
		//wyłaczenie przerwań
		__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
	}
}

void read_HCSR04(void){
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
	Delay_us(10);
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);

}
