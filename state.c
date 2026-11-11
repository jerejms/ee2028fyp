#include "main.h"
#include "state.h"
#include "button.h"
#include "buzzer.h"
#include "uart.h"
#include "sensors.h"
#include "telemetry.h"
#include "oled.h"

#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.h"

#include <stdio.h>

#define SENSOR_INTERVAL_MS 1000U
#define BATTLE_LED_INTERVAL_MS 500U
#define ATTACK_LED_INTERVAL_MS 250U
#define BUZZER_INTERVAL_MS 500U
#define RESCUE_TIMEOUT_MS 10000U
#define SHOT_DURATION_MS 800U
#define OLED_INTERVAL_MS 100U
#define ORIENTATION_INTERVAL_MS 10U
#define UPSIDE_DOWN_GRAVITY_Z (-0.8f)

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

static uint8_t laserFiring = 0;

static uint32_t laserStartTime = 0;
static uint32_t lastOledTime = 0;
static uint32_t lastOrientationTime = 0;
//testing purposes
static uint32_t lastOrientationLogTime = 0;
//which sensor is highlighted (0-5)
static int selectedSensor = 0;
//0 = menu, 1 = showing readings.
static uint8_t viewingSensor = 0;
//latest readings
static SensorReadings sensorValues;

static const char *sensorNames[6] = {
    "TEMPERATURE",
    "PRESSURE",
    "HUMIDITY",
    "ACCELEROMETER",
    "GYROSCOPE",
    "MAGNETOMETER"
};

static const char *sensorUnits[6] = {
    "DEG C",
    "HPA",
    "PERCENT",
    "M PER S2",
    "DEG PER S",
    "UT"
};


static void DrawSensorValue(int y, const char *label, float value){
    char text[32];
    snprintf(text, sizeof(text), "%s %.2f", label, value);
    OLED_DrawString(0, y, text);
}


static void ShowStandbyScreen(void){
    OLED_Clear();
    if (viewingSensor == 0){
        OLED_DrawString(0, 0, "STANDBY MENU");
        for (int i = 0; i < 6; i++){
            int y = 12 + i * 8;
            OLED_DrawString(12, y, sensorNames[i]);
            if (i == selectedSensor){
                //small error beside selected sensor
                OLED_DrawLine(2, y + 1, 5, y + 3);
                OLED_DrawLine(5, y + 3, 2, y + 5);
            }
        }
    }
    else{
        OLED_DrawString(0, 0, sensorNames[selectedSensor]);
        switch (selectedSensor){
            case 0:
                DrawSensorValue(16, "", sensorValues.temperature);
                break;
            case 1:
                DrawSensorValue(16, "", sensorValues.pressure);
                break;
            case 2:
                DrawSensorValue(16, "", sensorValues.humidity);
                break;
            case 3:
                DrawSensorValue(12, "X", sensorValues.acceleration[0]);
                DrawSensorValue(22, "Y", sensorValues.acceleration[1]);
                DrawSensorValue(32, "Z", sensorValues.acceleration[2]);
                break;
            case 4:
                DrawSensorValue(12, "X", sensorValues.gyroscope[0]);
                DrawSensorValue(22, "Y", sensorValues.gyroscope[1]);
                DrawSensorValue(32, "Z", sensorValues.gyroscope[2]);
                break;
            case 5:
                DrawSensorValue(12, "X", sensorValues.magnetic[0]);
                DrawSensorValue(22, "Y", sensorValues.magnetic[1]);
                DrawSensorValue(32, "Z", sensorValues.magnetic[2]);
                break;
        }

        OLED_DrawString(0, 44, sensorUnits[selectedSensor]);
        OLED_DrawString(0, 56, "LEFT TO GO BACK");
    }

    OLED_Update();
}


// Called once for each new joystick press.
void State_OnJoystick(uint8_t key){
    if (currentState != STANDBY || terminated){
        return;
    }

    if (viewingSensor == 0){
    	//up
        if (key == 0){
            selectedSensor--;
            if (selectedSensor < 0){
                selectedSensor = 5;
            }
        }
        //down
        else if (key == 2){
            selectedSensor++;
            if (selectedSensor > 5){
                selectedSensor = 0;
            }
        }
        //centre
        else if (key == 4){
            Sensors_Read(&sensorValues);
            viewingSensor = 1;
        }
        else{
            return;
        }
    }
    else{
    	//left
        if (key == 1){
            viewingSensor = 0;
        }
        else{
            return;
        }
    }
    ShowStandbyScreen();
}

static void StartLaserAnimation(void){
    laserFiring = 1;
    laserStartTime = HAL_GetTick();

    lastOledTime = laserStartTime - OLED_INTERVAL_MS;
}

static void UpdateCombatScreen(void)
{
    if (currentState == STANDBY || terminated){
        return;
    }
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - lastOledTime) < OLED_INTERVAL_MS){
        return;
    }
    lastOledTime = now;
    if (currentState == BATTLE){
        uint32_t elapsed = now - laserStartTime;
        if (laserFiring && elapsed >= SHOT_DURATION_MS){
            laserFiring = 0;
        }

        if (laserFiring){
            OLED_Clear();

            OLED_DrawString(0, 0, "LASER FIRED");
            OLED_DrawLaserAnimation(elapsed);
            OLED_DrawCompactChargeBar(laserCharge);

            OLED_Update();
        }
        else{
            char text[24];

            snprintf(text, sizeof(text),
                     "ENERGY %u OF 10",
                     (unsigned int)laserCharge);

            OLED_Clear();

            OLED_DrawString(0, 0, "BATTLE MODE");
            OLED_DrawString(0, 20, text);
            OLED_DrawString(0, 36, "READY TO CHARGE");

            OLED_DrawCompactChargeBar(laserCharge);
            OLED_Update();
        }
    }
    else if (currentState == ATTACKED){
        uint32_t elapsed = now - rescueStartTime;
        int secondsLeft = 0;
        if (elapsed < RESCUE_TIMEOUT_MS){
            //initially show 10 seconds.
            secondsLeft =(int)((RESCUE_TIMEOUT_MS - elapsed + 999U) / 1000U);
        }
        OLED_ShowAttacked(secondsLeft, buzzerState);
    }
}

// things that need to do when entering state
static void EnterState(DroneState newState){
    uint32_t now = HAL_GetTick();
    //discard the extra button clicks in queue when changing modes
    Button_Clear();

    currentState = newState;
    laserFiring = 0;
    //refresh combat screen
    lastOledTime = now - OLED_INTERVAL_MS;

    lastSensorTime = now;
    lastLedTime = now;
    lastBuzzerTime = now;
    lastOrientationTime = now;

    buzzerState = 0;
    Buzzer_Off();

    if (currentState == STANDBY){
        BSP_LED_On(LED2);
        UART_SendText("Entering STANDBY MODE\r\n");
        selectedSensor = 0;
        viewingSensor = 0;
        ShowStandbyScreen();
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
        StartLaserAnimation();
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
            Sensors_Read(&sensorValues);
			Telemetry_Send(&sensorValues);
			if (viewingSensor){
				ShowStandbyScreen();
			}
        }
    }
    else if (currentState == BATTLE){
        if ((uint32_t)(now - lastLedTime)>= BATTLE_LED_INTERVAL_MS){
            lastLedTime = now;
            BSP_LED_Toggle(LED2);
        }
        if ((uint32_t)(now - lastOrientationTime) >= ORIENTATION_INTERVAL_MS)
        {
            uint32_t elapsedMs = (uint32_t)(now - lastOrientationTime);
            lastOrientationTime = now;

            float dt = elapsedMs / 1000.0f;

            float gravityZ = Sensors_ReadGravityZ(dt);
            // Diagnostic logging every 200 ms
            if ((uint32_t)(now - lastOrientationLogTime) >= 200U)
            {
            	lastOrientationLogTime = now;

                char message[96];

                snprintf(message, sizeof(message),
                         "GRAVITYZ=%.2f DT=%.3f\r\n",
                         gravityZ, dt);

                UART_SendText(message);
            }
            // Initial upside-down detection
            if (gravityZ < UPSIDE_DOWN_GRAVITY_Z)
            {
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
            OLED_Clear();
            OLED_DrawString(0, 16, "RESCUE FAILED");
            OLED_DrawString(0, 32, "SYSTEM TERMINATED");
            OLED_Update();
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
    UpdateCombatScreen();
}
