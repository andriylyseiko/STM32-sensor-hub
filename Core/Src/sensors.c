/*
 * sensors.c
 *
 *  Created on: Aug 30, 2024
 *      Author: alyseiko
 */


/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "main.h"
#include "cmsis_os.h"
#include "sensors.h"
#include "bmp280.h"

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

// SPI
#define RW_WRITE_BIT 0x7F
#define RW_READ_BIT 0x80

// BMP280
#define RAW_BUFF_SIZE 7

/* Private typedef -----------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* BMP280 */
#define BMP280_CS_LOW()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define BMP280_CS_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)


/* Private variables ---------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;

// HTU21 Sensor
uint8_t sHTU21_RX_Buffer[S_HTU21D_RX_BUFFER_SIZE];
uint8_t pDataTX = S_HTU21D_TEMP_CMD_NO_HOLD;

/* BMP280 */
struct bmp280_sensor_data bmp280_sensor;
uint8_t raw_data_buff[RAW_BUFF_SIZE];

/* Private function prototypes -----------------------------------------------*/

/* For SPI */
static void write_data(uint8_t *txData, uint16_t size);
static void read_data(uint8_t start_addr, uint8_t *buff, uint16_t size);
static void read_data_IT(uint8_t start_addr, uint8_t *buff, uint16_t size);


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
/*  ----------------------------------------------------------------------------*/


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
/*  ----------------------------------------------------------------------------*/


/* SPI Sensor (BMP280) - Pressure/Temperature -----------------------------------*/
void pressureSensorInit()
{
	// set default configs
	uint8_t txData[] = {BMP280_REG_RESET & SPI_RW_WRITE_BIT, 0x0, BMP280_REG_CTRL_MEAS & SPI_RW_WRITE_BIT, 0xFF};
	write_data(txData, sizeof(txData));

	// read callibration parameters
	uint8_t start_addr = BMP280_REG_TEMPT_CALIB_START;
	uint16_t size = 25;
	uint8_t buff[size];
	memset(buff, 0, sizeof(buff));
	read_data(start_addr, buff, size);

	// store callibration parameters
	load_calib_data_from_raw(&bmp280_sensor.calib_params, &buff[1]);
}

void triggerPressureMeasurements()
{
	uint8_t start_addr = BMP280_REG_PRESS_MSB;
	read_data_IT(start_addr, raw_data_buff, RAW_BUFF_SIZE);
}

int32_t processPressureMeasurements()
{
	load_raw_pressure_from_bytes(&bmp280_sensor, &raw_data_buff[1]);
	load_raw_temperature_from_bytes(&bmp280_sensor, &raw_data_buff[4]);

	compensate_measurements(&bmp280_sensor);

	return bmp280_sensor.compensated_data.pressure;
}

static void write_data(uint8_t *txData, uint16_t size)
{
	BMP280_CS_LOW();
	HAL_SPI_Transmit(&hspi1, txData, sizeof(txData), HAL_MAX_DELAY);
	BMP280_CS_HIGH();
}

static void read_data(uint8_t start_addr, uint8_t *buff, uint16_t size)
{
	uint8_t txData[1];
	txData[0] = start_addr | SPI_RW_READ_BIT;

	BMP280_CS_LOW();
	HAL_SPI_TransmitReceive(&hspi1, txData, buff, size, HAL_MAX_DELAY);
	BMP280_CS_HIGH();
}

static void read_data_IT(uint8_t start_addr, uint8_t *buff, uint16_t size)
{
	uint8_t txData[1];
	txData[0] = start_addr | SPI_RW_READ_BIT;

	BMP280_CS_LOW();
	HAL_SPI_TransmitReceive_IT(&hspi1, txData, buff, size);
}

/*  ----------------------------------------------------------------------------*/
