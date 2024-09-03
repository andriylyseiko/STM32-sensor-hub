/*
 * task_handler.c
 *
 *  Created on: Aug 30, 2024
 *      Author: alyseiko
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "cmsis_os.h"
#include "sensors.h"

/* Private define ------------------------------------------------------------*/
#define CMD_LEN 10

/* Private typedef -----------------------------------------------------------*/
typedef struct {
	uint8_t payload[CMD_LEN + 1];
	uint8_t len;
}command_t;

/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart2;

// queues handlers
extern osMessageQueueId_t dataQueueHandle;
extern osMessageQueueId_t printQueueHandle;

// tasks handlers
extern osThreadId_t cmd_taskHandle;
extern osThreadId_t menu_taskHandle;
extern osThreadId_t print_taskHandle;

extern osThreadId_t light_taskHandle;
extern osThreadId_t temp_taskHandle;
extern osThreadId_t hum_taskHandle;

extern osThreadId_t LED_taskHandle;

volatile float lux;
volatile float temp;


/* Private function prototypes -----------------------------------------------*/
void cmdHandlerTask(void *argument);
void menuTask(void *argument);
void printTask(void *argument);
void lightMeasureTask(void *argument);
extern void LEDRegulateTask(void *argument);

static int extractCommand(command_t *cmd);
static void processCommand(command_t *cmd);

/**
* @brief Function implementing the cmd_task thread.
* @param argument: Not used
* @retval None
*/
void cmdHandlerTask(void *argument)
{
	uint32_t flags;
	command_t cmd;

	while(1) {
		flags = osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);
		if (flags) {
			extractCommand(&cmd);
			processCommand(&cmd);
		}
	}
}

/**
 *
 */
static int extractCommand(command_t *cmd)
{
	uint8_t item;
	uint32_t count;
	osStatus_t status;

	count = osMessageQueueGetCount(dataQueueHandle);
	if (!count)
		return -1;

	uint8_t i = 0;
	cmd->len = 0;
	do {
		status = osMessageQueueGet(dataQueueHandle, &item, 0U, 0U);
		if (status == osOK)
			cmd->payload[i++] = (uint8_t)item;

	} while (item != '\n' );

	cmd->payload[i-1] = '\0'; // replace \n
	cmd->len = i - 1;

	return 0;
}

/**
 *
 */
static void processCommand(command_t *cmd)
{
	uint8_t option;

	if (cmd->len == 1) {
		option = cmd->payload[0] - 48; // ASCII to number
		switch (option) {
			case 0:
				osThreadFlagsSet(menu_taskHandle, 1);
				break;
			case 1:
				osThreadFlagsSet(light_taskHandle, 1);
				break;
			case 2:
				osThreadFlagsSet(temp_taskHandle, 1);
				break;
			case 3:
				osThreadFlagsSet(hum_taskHandle, 1);
				break;
			default:
				// TODO: print error (command don't exist)
				break;

		}
	}
}

/**
* @brief Function implementing the menu_task thread.
* @param argument: Not used
* @retval None
*/
void menuTask(void *argument)
{
	const char *menuMsg = "Menu\n";

	while (1) {

		// This task is called first by default and then it is waiting for next triggering
		osMessageQueuePut(printQueueHandle, &menuMsg, 0U, osWaitForever);
		osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);
	}
}

/**
* @brief Function implementing the Light_Task thread.
* @param argument: Not used
* @retval None
*/
void lightMeasureTask(void *argument)
{
	const char *waitMsg = "Measure Light...\n";
	char *resMsg;
	char outBuffer[16];
	resMsg = outBuffer;

	while (1) {
		osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);

		osMessageQueuePut(printQueueHandle, &waitMsg, 0U, osWaitForever);

		// trigger measuring
		triggerLightMeasuring();

		// wait for measurement completed (ADC Callback will notify when finished with flags = 2)
		osThreadFlagsWait(2, osFlagsWaitAny, osWaitForever);
		lux = getLightMeasuredValue();

		sprintf(outBuffer, "Lux: %.1f\n", lux);

		// Enqueue measured value for further printing
		osMessageQueuePut(printQueueHandle, &resMsg, 0U, osWaitForever);

		// notify ledRegulate for configuring LED brightness using PWM
		osThreadFlagsSet(LED_taskHandle, 1);
	}
}

/**
 *
 */
void temperatureMeasureTask(void *argument)
{
	uint32_t flags;
	const char *waitMsg = "Measure Temp...\n";
	char *resMsg;
	char outBuffer[16];
	resMsg = outBuffer;

	while (1) {
		osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);

		osMessageQueuePut(printQueueHandle, &waitMsg, 0U, osWaitForever);

		triggerTempMeasuring();

		// wait for measurement start result
		flags = osThreadFlagsWait(2 | 6, osFlagsWaitAny, osWaitForever);
		if (flags == 6) {
			// error occur, TODO: print error message
			continue;
		}

		startReceivingTempMeasurement();

		// wait for receiving completed (I2C Callback will notify when finished with flags = 4)
		flags = osThreadFlagsWait(4, osFlagsWaitAny, osWaitForever);
		if (flags == 6) {
			// error occur, TODO: print error message
			continue;
		}

		temp = getTempMeasuredValue();

		sprintf(outBuffer, "Temp: %.1f\n", temp);

		// Enqueue measured value for further printing
		osMessageQueuePut(printQueueHandle, &resMsg, 0U, osWaitForever);
	}
}

/**
 *
 */
void humidityMeasureTask(void *argument)
{
	while (1) {

	}
}

/**
  * @brief  Function implementing the PWM_Task thread.
  * @param  argument: Not used
  * @retval None
  */
void LEDRegulateTask(void *argument)
{
	uint16_t compareValue;
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	while (1) {
		osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);

		// todo: define those constants
		compareValue = (lux * 65535) / 600;

		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, compareValue);
	}
}

/**
* @brief Function implementing the print_task thread.
* @param argument: Not used
* @retval None
*/
void printTask(void *argument)
{
	uint32_t *msg;
	while (1) {
		osMessageQueueGet(printQueueHandle, &msg, 0U, osWaitForever);
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen((char*)msg), HAL_MAX_DELAY);
	}
}
