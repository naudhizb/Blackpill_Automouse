/*
 * ButtonCallback.c
 *
 *  Created on: Oct 4, 2023
 *      Author: Envie
 */



#include "main.h"
#include <stm32f4xx_hal.h>
#include <stdint.h>

extern int working_flag;
extern int expired_flag;
extern int detect_flag;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  UNUSED(GPIO_Pin);

  if(GPIO_Pin == B0_Pin)
  {
	  static uint32_t pressed = 0;
	  static uint32_t released = 0;
	  const uint32_t debounce_time = 10;
	  GPIO_PinState B0 = HAL_GPIO_ReadPin(B0_GPIO_Port, B0_Pin);
	  if(B0 == GPIO_PIN_RESET)
	  { // pressed
		  pressed = HAL_GetTick();
	  }
	  else
	  { // released
		  released = HAL_GetTick();
		  uint32_t button_time = (released - pressed);
		  if(button_time > debounce_time)
		  {
			  working_flag = !working_flag;
			  if(working_flag)
				  expired_flag = 0;
		  }
		  else
		  {
			  // chattering, ignore
		  }
	  }
  }
  if(GPIO_Pin == RADAR_OUT_Pin)
  {
	  GPIO_PinState B10 = HAL_GPIO_ReadPin(RADAR_OUT_GPIO_Port, RADAR_OUT_Pin);
	  detect_flag  = !!B10;
  }
}
