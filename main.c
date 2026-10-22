  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) EE2028 Teaching Team
  ******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.h"
#include <stdio.h>

extern void initialise_monitor_handles(void);	// for semi-hosting support (printf)


//Pushbutton & States start here
//State values of our vehicle
#define STANDBY 0
#define BATTLE 1
#define ATTACKED 2

//variables used for pushbutton
volatile int state = STANDBY;
volatile int waitingSecondPress = 0;
volatile uint32_t firstPressTime = 0;
volatile uint32_t chargeRequests = 0;


int main(void)
{
	//This part is skeleton
	initialise_monitor_handles(); // for semi-hosting support (printf)
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Peripheral initializations using BSP functions */
    BSP_ACCELERO_Init();
    BSP_GYRO_Init();
    BSP_MAGNETO_Init();
    BSP_TSENSOR_Init();
    BSP_PSENSOR_Init();
    BSP_HSENSOR_Init();

	//PB initialization here
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
	int lastState = -1;

	//time checking
	uint32_t lastSensorTime = HAL_GetTick();
	while (1)
	{
		//remember interrupt setting
		uint32_t savedMask = __get_PRIMASK();
		//temporarily pause the interrupts
		__disable_irq();

		//PB code starts here
		if (waitingSecondPress && (uint32_t)(HAL_GetTick() - firstPressTime) >= 500){
			waitingSecondPress = 0;

			if(state == BATTLE){
				chargeRequests++;
			}
		}

		int currentState = state;
		uint32_t charges = chargeRequests;
		chargeRequests = 0;

		//restore interrupt setting
		__set_PRIMASK(savedMask);

		if(currentState != lastState){
			if(currentState == STANDBY){
				printf("Drone is on Standby State\n");
			}
			else if(currentState == BATTLE){
				printf("Drone is on Battle State\n");
			}
			else{
				printf("Drone is under Attack!");
			}
			lastState = currentState;
		}

		if(charges > 0){
			printf("Charge +3/10, %lu single presses \n", (unsigned long)charges);
		}

		if(currentState == STANDBY){
			uint32_t now = HAL_GetTick();
			if((uint32_t)(now - lastSensorTime) >= 1000){

				lastSensorTime = now;

		        float accel_data[3];
		        int16_t accel_data_i16[3] = { 0 };
		        BSP_ACCELERO_AccGetXYZ(accel_data_i16);

		        // Convert acceleration from mg to m/s^2.
		        accel_data[0] = (float)accel_data_i16[0] * (9.8/1000.0f);
		        accel_data[1] = (float)accel_data_i16[1] * (9.8/1000.0f);
		        accel_data[2] = (float)accel_data_i16[2] * (9.8/1000.0f);

		        float temp_data = BSP_TSENSOR_ReadTemp();
		        float pressure_data = BSP_PSENSOR_ReadPressure();
		        float humidity_data = BSP_HSENSOR_ReadHumidity();

		        float gyro_data[3];
		        float gyro_data_float[3] = { 0 };
		        BSP_GYRO_GetXYZ(gyro_data_float);

		        // Values are in millidegrees per second.
		        gyro_data[0] = gyro_data_float[0];
		        gyro_data[1] = gyro_data_float[1];
		        gyro_data[2] = gyro_data_float[2];

		        float magneto_data[3];
		        int16_t magneto_data_i16[3] = { 0 };
		        BSP_MAGNETO_GetXYZ(magneto_data_i16);

		        // Convert magnetic field from milligauss to microtesla.
		        magneto_data[0] = (float)magneto_data_i16[0]/10;
		        magneto_data[1] = (float)magneto_data_i16[1]/10;
		        magneto_data[2] = (float)magneto_data_i16[2]/10;

		        printf("T: %.2f degC | P: %.2f hPa | H: %.2f %%RH\r\n"
		               "A: X=%.2f Y=%.2f Z=%.2f m/s^2\r\n"
		               "G: X=%.2f Y=%.2f Z=%.2f mdeg/s\r\n"
		               "M: X=%.2f Y=%.2f Z=%.2f uT\r\n\r\n",
		               temp_data, pressure_data, humidity_data,
		               accel_data[0], accel_data[1], accel_data[2],
		               gyro_data[0], gyro_data[1], gyro_data[2],
		               magneto_data[0], magneto_data[1], magneto_data[2]);
		    }

		}
		else if(currentState == BATTLE){
			//battle code here
		}
		else if(currentState == ATTACKED){
			//10 seconds rescue timeout
		}

	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN){
	static uint32_t lastPressTime = 0;
	static int havePreviousPress = 0;
	if(GPIO_PIN != USER_BUTTON_PIN){
		return;
	}
	uint32_t now = HAL_GetTick();

	if (havePreviousPress && (uint32_t)(now - lastPressTime) < 50){
	    return;
	}

	havePreviousPress = 1;
	lastPressTime = now;

	//resolve an older single press if main was busy
	if(waitingSecondPress && (uint32_t)(now - firstPressTime) >= 500){
		waitingSecondPress = 0;

		if(state == BATTLE){
			chargeRequests++;
		}
	}

	if(waitingSecondPress){
		//double press
		waitingSecondPress = 0;
		if(state == BATTLE){
			state = STANDBY;
		}
		else{
			state = BATTLE; //from standby or rescue successful
		}
	}
	else{
		firstPressTime = now;
		waitingSecondPress = 1;
	}
}
