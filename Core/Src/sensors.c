/*
 * sensors.c
 *
 *  Created on: Aug 30, 2024
 *      Author: alyseiko
 */


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "sensors.h"

/* Private define ------------------------------------------------------------*/
// Common
#define S_VOLTAGE_REF 3
#define MICRO_CONVERSION_FACTOR 1000000.0f

// TEMPT6000 sensor
#define S_ADC_RESOLUTION 4096
#define S_INTERNAL_RESISTANCE 10000.0f // 10K
#define S_MICROAMPS_TO_LUX_COF 2.0f // 2 microamps = 1 lux

/* Private typedef -----------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/

extern ADC_HandleTypeDef hadc1;

/* Private function prototypes -----------------------------------------------*/



void triggerLightMeasuring()
{
	HAL_ADC_Start_IT(&hadc1);
}


float getLightMeasuredValue()
{
	uint32_t adcValue;
	float voltage;
	float result;

	adcValue = HAL_ADC_GetValue(&hadc1);

	voltage = (float) (adcValue * S_VOLTAGE_REF ) / S_ADC_RESOLUTION;
	result = (voltage / S_INTERNAL_RESISTANCE) * MICRO_CONVERSION_FACTOR * S_MICROAMPS_TO_LUX_COF;

	HAL_ADC_Stop_IT(&hadc1);

	return (float) result;
}
