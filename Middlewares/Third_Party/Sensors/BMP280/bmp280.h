/*
 * bmp280.h
 *
 *  Created on: Sep 5, 2024
 *      Author: alyseiko
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
struct bmp280_measurements
{
	int32_t	temperature;
	int32_t	pressure;
};

struct bmp280_calib_params
{
    /*! Calibration coefficients for the temperature sensor */
    uint16_t	dig_t1;
    int16_t		dig_t2;
    int16_t		dig_t3;

    /*! Calibration coefficients for the pressure sensor */
    uint16_t 	dig_p1;
    int16_t 	dig_p2;
    int16_t 	dig_p3;
    int16_t 	dig_p4;
    int16_t 	dig_p5;
    int16_t 	dig_p6;
    int16_t 	dig_p7;
    int16_t 	dig_p8;
    int16_t 	dig_p9;

    /*! Variable to store the intermediate temperature coefficient */
    int32_t 	t_fine;
};

/*
 * Device structure
 *
 */
struct bmp280_sensor_data
{
	// TODO: add config structure, add ctrl_meas structure, status, id
	struct bmp280_measurements	raw_data;
	struct bmp280_measurements	compensated_data;
	struct bmp280_calib_params	calib_params;
};

/* Exported constants --------------------------------------------------------*/

/* BMP280 Registers */
#define BMP280_REG_PRESS_MSB			0xF7
#define BMP280_REG_CTRL_MEAS			0xF4
#define BMP280_REG_CONFIG				0xF5
#define BMP280_REG_ID					0xD0
#define BMP280_REG_CALLIB0				0x88
#define BMP280_REG_RESET				0xE0
#define BMP280_REG_PRESS_CALIB_START	0x8E
#define BMP280_REG_TEMPT_CALIB_START	0x88

/* general purpose */
#define SPI_RW_WRITE_BIT				0x7F
#define SPI_RW_READ_BIT					0x80

/* Exported macro ------------------------------------------------------------*/

/* Macro to combine two 8 bit data's to form a 16 bit data */
#define BMP280_BUILD_16BIT_FROM_BYTES(msb, lsb)				(((uint16_t)(msb) << 8) | (uint16_t)(lsb))

/* Macro to combine two 8 bit and one 4 bit data's to form a 20 bit data */
#define BMP280_BUILD_20BIT_FROM_BYTES(msb, lsb, xlsb)		(((uint32_t)(msb) << 12) | ((uint32_t)(lsb) << 4) | ((uint32_t)(xlsb) >> 4))


/* Exported functions prototypes ---------------------------------------------*/

uint32_t compensate_pressure(struct bmp280_measurements *raw_data, struct bmp280_calib_params *calib_params);
int32_t compensate_temperature(struct bmp280_measurements *raw_data, struct bmp280_calib_params *calib_params);
void load_calib_data_from_raw(struct bmp280_calib_params *calib_data, uint8_t *raw_data);
void compensate_measurements(struct bmp280_sensor_data *bmp280_s);
void load_raw_pressure_from_bytes(struct bmp280_sensor_data *bmp280_s, uint8_t *bytes);
void load_raw_temperature_from_bytes(struct bmp280_sensor_data *bmp280_s, uint8_t *bytes);


#endif /* INC_BMP280_H_ */
