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
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.h"
#include <stdio.h>
#include "string.h"

extern void initialise_monitor_handles(void);	// for semi-hosting support (printf)


//Pushbutton & States start here
//State values of our vehicle
#define STANDBY 0
#define BATTLE 1
#define ATTACKED 2

//UART initialization
static void UART1_Init(void);

UART_HandleTypeDef huart1;

//variables used for pushbutton
volatile int state = STANDBY;
volatile int waitingSecondPress = 0;
volatile uint32_t firstPressTime = 0;
volatile uint32_t chargeRequests = 0;
volatile uint32_t rescueStartTime = 0;
volatile uint8_t rescueSucceeded;



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
    BSP_LED_Init(LED2);
    /* UART initialization  */
    UART1_Init();
    char messagePrint[64];
	//PB initialization here
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
	int lastState = -1;

	//time checking
	uint32_t lastSensorTime = HAL_GetTick();
	int laserCharge = 0; // 4 means is 4/10
	uint32_t lastLedTime = HAL_GetTick();

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
			    BSP_LED_On(LED2);
			    snprintf(messagePrint, sizeof(messagePrint), "Entering STANDBY MODE\r\n");
			    HAL_UART_Transmit(&huart1, (uint8_t*)messagePrint,strlen(messagePrint), 0xFFFF);
			    printf("Drone is on Standby\r\n");
			}
			else if(currentState == BATTLE){
			    BSP_LED_On(LED2);
			    lastLedTime = HAL_GetTick();
			    snprintf(messagePrint, sizeof(messagePrint),"Entering BATTLE MODE\r\n");
			    HAL_UART_Transmit(&huart1, (uint8_t*)messagePrint,strlen(messagePrint), 0xFFFF);
			    printf("Drone is ready for battle against Mr.Pandi!\r\n");
			}
			else if(currentState == ATTACKED){
				rescueStartTime = HAL_GetTick();
				lastSensorTime = rescueStartTime;
				rescueSucceeded = 0;
				BSP_LED_Off(LED2);
				printf("Drone is under Attack!\r\n");
			}
			lastState = currentState;
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
		               "G: X=%.2f Y=%.2f Z=%.2f deg/s\r\n"
		               "M: X=%.2f Y=%.2f Z=%.2f uT\r\n\r\n",
		               temp_data, pressure_data, humidity_data,
		               accel_data[0], accel_data[1], accel_data[2],
		               gyro_data[0], gyro_data[1], gyro_data[2],
		               magneto_data[0], magneto_data[1], magneto_data[2]);
		    }

		}
		else if(currentState == BATTLE){
			uint32_t now = HAL_GetTick();

		    // Toggle every 500 ms: one complete ON/OFF cycle per second.
		    if ((uint32_t)(now - lastLedTime) >= 500)
		    {
		        lastLedTime = now;
		        BSP_LED_Toggle(LED2);
		    }

		    // Process each single press recorded by the button code.
		    while (charges > 0)
		    {
		        charges--;
		        laserCharge += 3;

		        // Fire immediately when at least 5/10 energy is available.
		        if (laserCharge >= 5)
		        {
		            laserCharge -= 5;

		            snprintf(messagePrint, sizeof(messagePrint),
		                     "Laser fired! Remaining charge: %d/10\r\n",
		                     laserCharge);
		        }
		        else
		        {
		            snprintf(messagePrint, sizeof(messagePrint),
		                     "Laser charge: %d/10\r\n",
		                     laserCharge);
		        }

		        HAL_UART_Transmit(&huart1, (uint8_t*)messagePrint,
		                          strlen(messagePrint), 0xFFFF);
		    }
		}
		else if(currentState == ATTACKED){
			//10 seconds rescue timeout
			//wait for rescue in 10 seconds, LED blink at 2 Hz
			// double press means rescue, then drone goes back to BATTLE_MODE.
			//if rescue fail, send last message to Command Centre, and then terminate program

			uint32_t now = HAL_GetTick();

			//rescue succeeded: return to battle
			if (rescueSucceeded) {
				rescueStartTime = 0;
				rescueSucceeded = 0;

				BSP_LED_Off(LED2);

				uint32_t savedMask = __get_PRIMASK();
				__disable_irq();

				state = BATTLE;
				rescueStartTime = 0;

				__set_PRIMASK(savedMask);

				printf("Rescue successful! Returning to BATTLE. \r\n");
			}

			//timeout after 10 seconds
			else if ((uint32_t)(now - rescueStartTime) >= 10000) {
				BSP_LED_Off(LED2);

				printf("RESCUE FAILED: 10-second time. Kontol!\r\n");

				//TODO: send final message to command centre here.
				snprintf(messagePrint, sizeof(messagePrint),
									"RESCUE FAILED: 10-second timeout. Kontol!\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)messagePrint, strlen(messagePrint),0xFFFF); //Sending in normal mode
				//stop normal program execution.
				while (1) {
					//remain terminated.
				}
			}

			//blink at 2 Hz: toggle every 250 ms
			else if ((uint32_t)(now - lastSensorTime) >= 250) {
				lastSensorTime = now;
				BSP_LED_Toggle(LED2);
			}
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
		} else if (state == STANDBY) {
			state = BATTLE; //from standby or rescue successful
		} else if (state == ATTACKED) {
			if ((uint32_t)(now - rescueStartTime) < 10000) {
				rescueSucceeded = 1;
			}
		}
	}
	else{
		firstPressTime = now;
		waitingSecondPress = 1;
	}
}

static void UART1_Init(void)
{
		/* Pin configuration for UART. BSP_COM_Init() can do this automatically */
		__HAL_RCC_GPIOB_CLK_ENABLE();
         __HAL_RCC_USART1_CLK_ENABLE();

         GPIO_InitTypeDef GPIO_InitStruct = {0};
         GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
         GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
         GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
         GPIO_InitStruct.Pull = GPIO_NOPULL;
         GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
         HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

         /* Configuring UART1 */
         huart1.Instance = USART1;
         huart1.Init.BaudRate = 115200;
         huart1.Init.WordLength = UART_WORDLENGTH_8B;
         huart1.Init.StopBits = UART_STOPBITS_1;
         huart1.Init.Parity = UART_PARITY_NONE;
         huart1.Init.Mode = UART_MODE_TX_RX;
         huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
         huart1.Init.OverSampling = UART_OVERSAMPLING_16;
         huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
         huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
         if (HAL_UART_Init(&huart1) != HAL_OK)
         {
        	 while(1);
         }

}
