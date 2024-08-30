/*
 * task_handler.c
 *
 *  Created on: Aug 30, 2024
 *      Author: alyseiko
 */

#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim4;

/**
  * @brief  Function implementing the PWM_Task thread.
  * @param  argument: Not used
  * @retval None
  */
void PWM_For_LED_Task(void *argument)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	uint32_t compareValue = 0;
  /* Infinite loop */
  for(;;)
  {
	  if (compareValue <= 0) {
		  printf("PWM task - increasing\n");

		  while(compareValue < 65535) {
			  printf("Val+: %ld\n", compareValue);
			  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, compareValue);
			  compareValue += 100;
			  HAL_Delay(10);
		  }

	  } else {
		  printf("PWM task - decreasing\n");

		  while(compareValue > 0) {
			  printf("Val-: %ld\n", compareValue);
			  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, compareValue);
			  compareValue -= 100;
			  HAL_Delay(10);
		  }
	  }
	    printf("Test execution1\n");

    osDelay(5000);
  }
}

/**
* @brief Function implementing the Light_Task thread.
* @param argument: Not used
* @retval None
*/
void GetLightIntensityTask(void *argument)
{
	uint16_t refVoltage = 3;
	uint16_t resolution = 4096;
	uint32_t rawValue;
	float voltage;
	float lux = 0;

  for(;;)
  {
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 20);
	  rawValue = HAL_ADC_GetValue(&hadc1);

	  printf("Raw value: %lu (0-4095)\n", rawValue);

	  voltage = (float) (rawValue * refVoltage ) / resolution;
	  printf("Voltage: %.3f V\n", voltage);

    /*
      Convert to Lux
      10k Om - resistor. Then convert ampere to micro amperes
      and multiply by 2, as 2 microamps = 1 lux
     */
	  lux = (voltage / 10000.0) * 1000000.0 * 2.0; //
	  printf("Lux: %.1f\n", lux);


	  HAL_ADC_Stop(&hadc1);
	  osDelay(5000);
  }
}

/**
* @brief Function implementing the menu_task thread.
* @param argument: Not used
* @retval None
*/
void menuTask(void *argument)
{
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
}

/**
* @brief Function implementing the cmd_task thread.
* @param argument: Not used
* @retval None
*/
void cmdHandlerTask(void *argument)
{
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
}

/**
* @brief Function implementing the print_task thread.
* @param argument: Not used
* @retval None
*/
void printTask(void *argument)
{
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
}
