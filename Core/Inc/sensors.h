/*
 * sensors.h
 *
 *  Created on: Sep 2, 2024
 *      Author: alyseiko
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

/* ADC Sensor (TEMPT6000) - Light Intensity (Lux) ----------------------------*/
void triggerLightMeasuring();
float getLightMeasuredValue();

/* I2C Sensor (HTU21) - Temperature (Celsius) -----------------------------------*/
void triggerTempMeasuring();
void startReceivingTempMeasurement();
float getTempMeasuredValue();

/* SPI Sensor (BMP280) - Pressure/Temperature -----------------------------------*/
void pressureSensorInit();
void triggerPressureMeasurements();
int32_t processPressureMeasurements();

#endif /* INC_SENSORS_H_ */
