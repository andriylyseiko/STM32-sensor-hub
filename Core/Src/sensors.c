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
#define S_MICRO_CONVERSION_FACTOR 1000000.0f

// TEMPT6000 sensor
#define S_ADC_RESOLUTION 4096
#define S_INTERNAL_RESISTANCE 10000.0f // 10K
#define S_MICROAMPS_TO_LUX_COF 2.0f // 2 microamps = 1 lux

// HTU21 sensor
#define S_HTU21D_ADDR (0x40 << 1)
#define S_HTU21D_TEMP_CMD_NO_HOLD 0xF3
#define S_HTU21D_HUM_CMD_NO_HOLD 0xF5
#define S_HTU21D_RX_BUFFER_SIZE 3

/* Private typedef -----------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;

// HTU21 Sensor
uint8_t sHTU21_RX_Buffer[S_HTU21D_RX_BUFFER_SIZE];
uint8_t pDataTX = S_HTU21D_TEMP_CMD_NO_HOLD;

/* Private function prototypes -----------------------------------------------*/


/* ADC Sensor (TEMPT6000) - Light Intensity (Lux) ----------------------------*/

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
	result = (voltage / S_INTERNAL_RESISTANCE) * S_MICRO_CONVERSION_FACTOR * S_MICROAMPS_TO_LUX_COF;

	HAL_ADC_Stop_IT(&hadc1);

	return (float) result;
}

/* I2C Sensor (HTU21) - Temperature (Celsius) -----------------------------------*/
void triggerTempMeasuring()
{
	HAL_I2C_Master_Transmit_IT(&hi2c1, S_HTU21D_ADDR, &pDataTX, sizeof(pDataTX));
}

/**
 *
 */
void startReceivingTempMeasurement()
{
	HAL_I2C_Master_Receive_IT(&hi2c1, S_HTU21D_ADDR, sHTU21_RX_Buffer, S_HTU21D_RX_BUFFER_SIZE);
}

/**
 *
 */
float getTempMeasuredValue()
{
	uint16_t HTU21D_ADC_Raw;

	HTU21D_ADC_Raw = ((uint16_t)(sHTU21_RX_Buffer[0] << 8) | (sHTU21_RX_Buffer[1]));
	return (float)(HTU21D_ADC_Raw * 175.72 / 65536.00) - 46.85;
}
