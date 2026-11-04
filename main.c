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
#define BUZZER_PORT GPIOD
#define BUZZER_PIN  GPIO_PIN_14

uint32_t lastBuzzerTime = 0;
uint8_t buzzerState = 0;

//OLED
#define OLED_ADDR (0x3C << 1)
#define OLED_WIDTH  128
#define OLED_HEIGHT 64
#define OLED_PAGES  (OLED_HEIGHT / 8)

I2C_HandleTypeDef hi2c1;
static uint8_t oledBuffer[OLED_WIDTH * OLED_PAGES];

static void I2C1_Init(void);
static HAL_StatusTypeDef OLED_Command(uint8_t cmd);
static HAL_StatusTypeDef OLED_Init(void);
static HAL_StatusTypeDef OLED_TestPattern(void);
static HAL_StatusTypeDef OLED_Update(void);
static void OLED_Clear(void);
static void OLED_DrawPixel(int x, int y, uint8_t color);
static void OLED_DrawProgressBar(int x, int y, int charge);

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

	__HAL_RCC_GPIOD_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = BUZZER_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	HAL_GPIO_Init(BUZZER_PORT, &GPIO_InitStruct);

	// Buzzer initially OFF
	HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);

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
    I2C1_Init();
    char messagePrint[64];

	//PB initialization here
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
	int lastState = -1;

	//time checking
	uint32_t lastSensorTime = HAL_GetTick();
	int laserCharge = 0; // 4 means is 4/10
	int lastDisplayedCharge = -1;
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
				lastBuzzerTime = HAL_GetTick();
				buzzerState = 1;
				HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
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
	    	float accel_data[3];
	    	int16_t accel_data_i16[3] = { 0 };

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

		    if((uint32_t)(now - lastSensorTime) >= 1000){
		    	lastSensorTime = now;
		    	BSP_ACCELERO_AccGetXYZ(accel_data_i16);
		    	accel_data[2] = (float)accel_data_i16[2] * (9.8/1000.0f);

				if (accel_data[2] < -8.0f) {
					state = ATTACKED;
				}
		    }

		    if (laserCharge != lastDisplayedCharge)
		    {
		        OLED_Clear();
		        OLED_DrawProgressBar(5, 24, laserCharge);

		        HAL_StatusTypeDef status = OLED_Update();

		        if (status == HAL_OK)
		        {
		            lastDisplayedCharge = laserCharge;
		        }
		    }
		}
		else if(currentState == ATTACKED){
			//10 seconds rescue timeout
			//wait for rescue in 10 seconds, LED blink at 2 Hz
			// double press means rescue, then drone goes back to BATTLE_MODE.
			//if rescue fail, send last message to Command Centre, and then terminate program

			uint32_t now = HAL_GetTick();

			if((uint32_t)(now - lastSensorTime) >= 1000){
				lastSensorTime = now;
				snprintf(messagePrint, sizeof(messagePrint),
													"ANJING! DRONE ATTACKED FUCK FUCK FUCK!\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)messagePrint, strlen(messagePrint),0xFFFF); //Sending in normal mode

			}

			//rescue succeeded: return to battle
			if (rescueSucceeded) {

				//turn off buzzer
				HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
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

				//TURN OFF BUZZER BEFORE TERMINATION
				HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
				buzzerState = 0;

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
			if ((uint32_t)(now - lastLedTime) >= 250) {
				lastLedTime = now;
				BSP_LED_Toggle(LED2);
			}

			if ((uint32_t)(now - lastBuzzerTime) >= 500) {
				lastBuzzerTime = now;

				buzzerState = !buzzerState;

				HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, buzzerState ? GPIO_PIN_SET : GPIO_PIN_RESET);
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

static HAL_StatusTypeDef OLED_Command(uint8_t cmd)
{
    uint8_t data[2] = {0x80, cmd};

    return HAL_I2C_Master_Transmit(
        &hi2c1,
        OLED_ADDR,
        data,
        sizeof(data),
        100
    );
}

static HAL_StatusTypeDef OLED_Init(void)
{
    const uint8_t commands[] = {
        0xAE,       // Display OFF
        0xD5, 0x80, // Clock divide
        0xA8, 0x3F, // Multiplex ratio
        0xD3, 0x00, // Display offset
        0x40,       // Start line
        0x8D, 0x14, // Charge pump
        0xA1,       // Segment remap
        0xC8,       // COM scan direction
        0xDA, 0x12, // COM pins
        0x81, 0x7F, // Contrast
        0xD9, 0xF1, // Precharge
        0xDB, 0x40, // VCOMH
        0xA4,       // Display follows RAM
        0xA6,       // Normal display
        0xAF        // Display ON
    };

    for (uint32_t i = 0; i < sizeof(commands); i++)
    {
        HAL_StatusTypeDef status = OLED_Command(commands[i]);

        if (status != HAL_OK)
        {
            return status;
        }
    }

    return HAL_OK;
}

static HAL_StatusTypeDef OLED_TestPattern(void)
{
    HAL_StatusTypeDef status;

    // Display OFF, use RAM contents, normal display
    uint8_t setup[] = {
        0x00, 0xAE, 0xA4, 0xA6
    };

    status = HAL_I2C_Master_Transmit(
        &hi2c1, OLED_ADDR,
        setup, sizeof(setup), 100
    );

    if (status != HAL_OK) {
        return status;
    }

    // 1 control byte + 128 pixel bytes
    uint8_t pixels[129];
    pixels[0] = 0x40;

    // Write all 8 display pages
    for (uint8_t page = 0; page < 8; page++)
    {
        // Select page and reset column address
        uint8_t pageCmd[] = {
            0x00,
            (uint8_t)(0xB0 | page),
            0x00,
            0x10
        };

        status = HAL_I2C_Master_Transmit(
            &hi2c1, OLED_ADDR,
            pageCmd, sizeof(pageCmd), 100
        );

        if (status != HAL_OK) {
            return status;
        }

        // Page 0 white, remaining pages black
        memset(&pixels[1],
               (page == 0) ? 0xFF : 0x00,
               128);

        status = HAL_I2C_Master_Transmit(
            &hi2c1, OLED_ADDR,
            pixels, sizeof(pixels), 200
        );

        if (status != HAL_OK) {
            return status;
        }
    }

    // Turn display ON
    uint8_t displayOn[] = {0x00, 0xAF};

    return HAL_I2C_Master_Transmit(
        &hi2c1, OLED_ADDR,
        displayOn, sizeof(displayOn), 100
    );
}


static void OLED_Clear(void)
{
    memset(oledBuffer, 0, sizeof(oledBuffer));
}

static void OLED_DrawPixel(int x, int y, uint8_t color)
{
    if (x < 0 || x >= OLED_WIDTH ||
        y < 0 || y >= OLED_HEIGHT)
    {
        return;
    }

    uint16_t index = x + (y / 8) * OLED_WIDTH;
    uint8_t mask = (uint8_t)(1U << (y % 8));

    if (color)
    {
        oledBuffer[index] |= mask;
    }
    else
    {
        oledBuffer[index] &= (uint8_t)~mask;
    }
}

static HAL_StatusTypeDef OLED_Update(void)
{
    HAL_StatusTypeDef status;

    // Display follows RAM, normal display mode
    uint8_t setup[] = {
        0x00, 0xA4, 0xA6
    };

    status = HAL_I2C_Master_Transmit(
        &hi2c1, OLED_ADDR,
        setup, sizeof(setup), 100
    );

    if (status != HAL_OK) {
        return status;
    }

    uint8_t packet[OLED_WIDTH + 1];

    // Control byte: display data
    packet[0] = 0x40;

    // Transfer all 8 pages
    for (uint8_t page = 0; page < OLED_PAGES; page++)
    {
        // Select page and reset column address
        uint8_t pageCmd[] = {
            0x00,
            (uint8_t)(0xB0 | page),
            0x00,
            0x10
        };

        status = HAL_I2C_Master_Transmit(
            &hi2c1, OLED_ADDR,
            pageCmd, sizeof(pageCmd), 100
        );

        if (status != HAL_OK) {
            return status;
        }

        // Copy one page from framebuffer
        memcpy(
            &packet[1],
            &oledBuffer[page * OLED_WIDTH],
            OLED_WIDTH
        );

        // Send 128 pixels columns for this page
        status = HAL_I2C_Master_Transmit(
            &hi2c1, OLED_ADDR,
            packet, sizeof(packet), 200
        );

        if (status != HAL_OK) {
            return status;
        }
    }

    return HAL_OK;
}

static void OLED_DrawProgressBar(int x, int y, int charge)
{
    // Limit charge to 0-10
    if (charge < 0) charge = 0;
    if (charge > 10) charge = 10;

    const int segmentWidth = 10;
    const int segmentHeight = 16;
    const int gap = 2;

    for (int segment = 0; segment < 10; segment++)
    {
        int startX = x + segment * (segmentWidth + gap);

        for (int dy = 0; dy < segmentHeight; dy++)
        {
            for (int dx = 0; dx < segmentWidth; dx++)
            {
                int border =
                    (dx == 0 || dx == segmentWidth - 1 ||
                     dy == 0 || dy == segmentHeight - 1);

                int filled =
                    (segment < charge) &&
                    (dx > 1 && dx < segmentWidth - 2) &&
                    (dy > 1 && dy < segmentHeight - 2);

                OLED_DrawPixel(
                    startX + dx,
                    y + dy,
                    (border || filled) ? 1 : 0
                );
            }
        }
    }
}

static void I2C1_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // PB8 = I2C1_SCL, PB9 = I2C1_SDA
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    hi2c1.Instance = I2C1;

    // Initial 100 kHz candidate for 4 MHz kernel clock
    hi2c1.Init.Timing = 0x00420F13;

    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        while (1) {
        }
    }

    HAL_I2CEx_ConfigAnalogFilter(
        &hi2c1,
        I2C_ANALOGFILTER_ENABLE
    );
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
