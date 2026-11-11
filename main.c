  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) EE2028 Teaching Team
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.h"
#include <stdio.h>
#include "string.h"

//our structure (clean code)
#include "buzzer.h"
#include "joystick.h"
#include "uart.h"
#include "sensors.h"
#include "button.h"
#include "state.h"

extern void initialise_monitor_handles(void);	// for semi-hosting support (printf)

// I2C bus used by the Grove 5-Way Switch
I2C_HandleTypeDef hi2c1;
static void I2C1_Init(void);

int main(void)
{
    HAL_Init();

    Buzzer_Init();
    Sensors_Init();
    BSP_LED_Init(LED2);

    UART1_Init();
    I2C1_Init();

    //set initial state before button interrupts
    State_Init();
    Button_Init();

    char messagePrint[128];
    uint32_t lastJoystickTime = HAL_GetTick();
    uint8_t joystickButtons[5] = {0};
    uint8_t previousButtons[5] = {0};

    while (1)
    {
        State_Update();
        if (State_IsTerminated()){
            continue;
        }

        /* Read the joystick every 50 ms. */
        uint32_t now = HAL_GetTick();

        if ((uint32_t)(now - lastJoystickTime) >= 50U){
            lastJoystickTime = now;

            if (Joystick_ReadButtons(&hi2c1, joystickButtons) == HAL_OK){
                for (int i = 0; i < 5; i++){
                    if (joystickButtons[i] != previousButtons[i]){
                        snprintf(
                            messagePrint,
                            sizeof(messagePrint),
                            "JOYSTICK KEY %c: %s\r\n",
                            'A' + i,
                            joystickButtons[i] ? "PRESSED" : "RELEASED"
                        );
                        UART_SendText(messagePrint);
                        previousButtons[i] = joystickButtons[i];
                    }
                }
            }
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN){
    Button_OnInterrupt(GPIO_PIN);
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
