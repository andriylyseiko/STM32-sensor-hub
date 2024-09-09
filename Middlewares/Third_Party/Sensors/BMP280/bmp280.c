/*
 * bmp280.c
 *
 *  Created on: Sep 5, 2024
 *      Author: alyseiko
 */


#include "bmp280.h"

void compensate_measurements(struct bmp280_sensor_data *bmp280_s)
{
	bmp280_s->compensated_data.temperature = compensate_temperature(&bmp280_s->raw_data, &bmp280_s->calib_params);
	bmp280_s->compensated_data.pressure = compensate_pressure(&bmp280_s->raw_data, &bmp280_s->calib_params);
}

void load_calib_data_from_raw(struct bmp280_calib_params *calib_data, uint8_t *raw_data)
{
	calib_data->dig_t1 = BMP280_BUILD_16BIT_FROM_BYTES(raw_data[1], raw_data[0]);
	calib_data->dig_t2 = (int16_t)BMP280_BUILD_16BIT_FROM_BYTES(raw_data[3], raw_data[2]);
	calib_data->dig_t3 = (int16_t)BMP280_BUILD_16BIT_FROM_BYTES(raw_data[5], raw_data[4]);
	calib_data->dig_p1 = BMP280_BUILD_16BIT_FROM_BYTES(raw_data[7], raw_data[6]);
	calib_data->dig_p2 = (int16_t)BMP280_BUILD_16BIT_FROM_BYTES(raw_data[9], raw_data[8]);
	calib_data->dig_p3 = (int16_t)BMP280_BUILD_16BIT_FROM_BYTES(raw_data[11], raw_data[10]);
	calib_data->dig_p4 = (int16_t)BMP280_BUILD_16BIT_FROM_BYTES(raw_data[13], raw_data[12]);
	calib_data->dig_p5 = (int16_t)BMP280_BUILD_16BIT_FROM_BYTES(raw_data[15], raw_data[14]);
	calib_data->dig_p6 = (int16_t)BMP280_BUILD_16BIT_FROM_BYTES(raw_data[17], raw_data[16]);
	calib_data->dig_p7 = (int16_t)BMP280_BUILD_16BIT_FROM_BYTES(raw_data[19], raw_data[18]);
	calib_data->dig_p8 = (int16_t)BMP280_BUILD_16BIT_FROM_BYTES(raw_data[21], raw_data[20]);
	calib_data->dig_p9 = (int16_t)BMP280_BUILD_16BIT_FROM_BYTES(raw_data[23], raw_data[22]);
}

void load_raw_pressure_from_bytes(struct bmp280_sensor_data *bmp280_s, uint8_t *bytes)
{
	bmp280_s->raw_data.pressure = BMP280_BUILD_20BIT_FROM_BYTES(bytes[0], bytes[1], bytes[2]);
}

void load_raw_temperature_from_bytes(struct bmp280_sensor_data *bmp280_s, uint8_t *bytes)
{
	bmp280_s->raw_data.temperature = BMP280_BUILD_20BIT_FROM_BYTES(bytes[0], bytes[1], bytes[2]);
}

/*
 * This function is used to compensate the raw pressure data and return
 * the compensated data in integer data type
 */
uint32_t compensate_pressure(struct bmp280_measurements *raw_data, struct bmp280_calib_params *calib_params)
{
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    uint32_t var5;
    uint32_t pressure;
    uint32_t pressure_min = 30000;
    uint32_t pressure_max = 110000;

    var1 = (((int32_t)calib_params->t_fine) / 2) - (int32_t)64000;
    var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t)calib_params->dig_p6);
    var2 = var2 + ((var1 * ((int32_t)calib_params->dig_p5)) * 2);
    var2 = (var2 / 4) + (((int32_t)calib_params->dig_p4) * 65536);
    var3 = (calib_params->dig_p3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
    var4 = (((int32_t)calib_params->dig_p2) * var1) / 2;
    var1 = (var3 + var4) / 262144;
    var1 = (((32768 + var1)) * ((int32_t)calib_params->dig_p1)) / 32768;

    /* Avoid exception caused by division by zero */
    if (var1)
    {
        var5 = (uint32_t)((uint32_t)1048576) - raw_data->pressure;
        pressure = ((uint32_t)(var5 - (uint32_t)(var2 / 4096))) * 3125;

        if (pressure < 0x80000000)
        {
            pressure = (pressure << 1) / ((uint32_t)var1);
        }
        else
        {
            pressure = (pressure / (uint32_t)var1) * 2;
        }

        var1 = (((int32_t)calib_params->dig_p9) * ((int32_t)(((pressure / 8) * (pressure / 8)) / 8192))) / 4096;
        var2 = (((int32_t)(pressure / 4)) * ((int32_t)calib_params->dig_p8)) / 8192;
        pressure = (uint32_t)((int32_t)pressure + ((var1 + var2 + calib_params->dig_p7) / 16));

        if (pressure < pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else
    {
        pressure = pressure_min;
    }

    return pressure;
}

/*
 * This function is used to compensate the raw temperature data and
 * return the compensated temperature data in integer data type.
 */
int32_t compensate_temperature(struct bmp280_measurements *raw_data, struct bmp280_calib_params *calib_params)
{
    int32_t var1;
    int32_t var2;
    int32_t temperature;
    int32_t temperature_min = -4000;
    int32_t temperature_max = 8500;

    var1 = (int32_t)((raw_data->temperature / 8) - ((int32_t)calib_params->dig_t1 * 2));
    var1 = (var1 * ((int32_t)calib_params->dig_t2)) / 2048;
    var2 = (int32_t)((raw_data->temperature / 16) - ((int32_t)calib_params->dig_t1));
    var2 = (((var2 * var2) / 4096) * ((int32_t)calib_params->dig_t3)) / 16384;
    calib_params->t_fine = var1 + var2;
    temperature = (calib_params->t_fine * 5 + 128) / 256;

    if (temperature < temperature_min)
    {
        temperature = temperature_min;
    }
    else if (temperature > temperature_max)
    {
        temperature = temperature_max;
    }

    return temperature;
}

