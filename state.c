#include "main.h"
#include "state.h"
#include "button.h"
#include "buzzer.h"
#include "uart.h"
#include "sensors.h"
#include "telemetry.h"

#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.h"

#include <stdio.h>

#define SENSOR_INTERVAL_MS 1000U
#define BATTLE_LED_INTERVAL_MS 500U
#define ATTACK_LED_INTERVAL_MS 250U
#define BUZZER_INTERVAL_MS 500U
#define RESCUE_TIMEOUT_MS 10000U

#define UPSIDE_DOWN_Z_THRESHOLD (-8.0f) //for testing purposes only

typedef enum{
    STANDBY,
    BATTLE,
    ATTACKED
} DroneState;

static DroneState currentState = STANDBY;

static uint8_t laserCharge = 0;
static uint8_t terminated = 0;
static uint8_t buzzerState = 0;

static uint32_t lastSensorTime = 0;
static uint32_t lastLedTime = 0;
static uint32_t lastBuzzerTime = 0;
static uint32_t rescueStartTime = 0;


/* Perform the actions needed when entering a mode. */
static void EnterState(DroneState newState){
    uint32_t now = HAL_GetTick();
    /* Discard pending button presses from the previous mode. */
    Button_Clear();

    currentState = newState;

    lastSensorTime = now;
    lastLedTime = now;
    lastBuzzerTime = now;

    buzzerState = 0;
    Buzzer_Off();

    if (currentState == STANDBY){
        BSP_LED_On(LED2);
        UART_SendText("Entering STANDBY MODE\r\n");
    }
    else if (currentState == BATTLE){
        BSP_LED_On(LED2);
        UART_SendText("Entering BATTLE MODE\r\n");
    }
    else{
        rescueStartTime = now;
        BSP_LED_Off(LED2);
        buzzerState = 1;
        Buzzer_On();
        UART_SendText("Drone Was Attacked!\r\n");
    }
}


/* Add energy for one confirmed single press. */
static void ChargeLaser(void){
    char message[96];
    laserCharge += 3;
    if (laserCharge > 10){
        laserCharge = 10;
    }
    while (laserCharge >= 5){
        laserCharge -= 5;
        UART_SendText("Laser fired!\r\n");
    }
    snprintf(message, sizeof(message),
             "Energy remaining: %u/10\r\n",
             (unsigned int)laserCharge);
    UART_SendText(message);
}


void State_Init(void){
    laserCharge = 0;
    terminated = 0;
    EnterState(STANDBY);
}


uint8_t State_IsTerminated(void){
    return terminated;
}


void State_Update(void){
    if (terminated){
        return;
    }

    ButtonEvent event;
    uint32_t pressTime;

    /* Handle button events before running the mode. */
    while ((event = Button_GetEvent(&pressTime)) != BUTTON_NONE){
        if (event == BUTTON_DOUBLE){
            if (currentState == STANDBY){
                EnterState(BATTLE);
            }
            else if (currentState == BATTLE){
                EnterState(STANDBY);
            }
            else if ((uint32_t)(pressTime - rescueStartTime) < RESCUE_TIMEOUT_MS){
                EnterState(BATTLE);
                UART_SendText("Rescue successful!\r\n");
            }
        }
        else if (event == BUTTON_SINGLE && currentState == BATTLE){
            ChargeLaser();
        }
    }

    uint32_t now = HAL_GetTick();

    if (currentState == STANDBY){
        if ((uint32_t)(now - lastSensorTime) >= SENSOR_INTERVAL_MS){
            lastSensorTime = now;
            SensorReadings readings;
            Sensors_Read(&readings);
            Telemetry_Send(&readings);
        }
    }
    else if (currentState == BATTLE){
        if ((uint32_t)(now - lastLedTime)>= BATTLE_LED_INTERVAL_MS){
            lastLedTime = now;
            BSP_LED_Toggle(LED2);
        }
        if ((uint32_t)(now - lastSensorTime)>= SENSOR_INTERVAL_MS){
            lastSensorTime = now;
            float accelerationZ = Sensors_ReadAccelerationZ();
            if (accelerationZ < UPSIDE_DOWN_Z_THRESHOLD){
                EnterState(ATTACKED);
            }
        }
    }
    else if (currentState == ATTACKED){
        /* Check timeout before sending another periodic message. */
        if ((uint32_t)(now - rescueStartTime) >= RESCUE_TIMEOUT_MS){
            terminated = 1;
            buzzerState = 0;
            Buzzer_Off();
            BSP_LED_Off(LED2);
            UART_SendText("Rescue failed. System terminated.\r\n");
            return;
        }
        if ((uint32_t)(now - lastSensorTime) >= SENSOR_INTERVAL_MS){
            lastSensorTime = now;
            UART_SendText("Drone Was Attacked!\r\n");
        }

        if ((uint32_t)(now - lastLedTime) >= ATTACK_LED_INTERVAL_MS){
            lastLedTime = now;
            BSP_LED_Toggle(LED2);
        }

        if ((uint32_t)(now - lastBuzzerTime) >= BUZZER_INTERVAL_MS){
            lastBuzzerTime = now;
            buzzerState = !buzzerState;

            if (buzzerState){
                Buzzer_On();
            }
            else{
                Buzzer_Off();
            }
        }
    }
}
