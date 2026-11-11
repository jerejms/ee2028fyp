#include "main.h"
#include "button.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.h"

#define DEBOUNCE_MS 50U
#define DOUBLE_PRESS_MS 500U
#define PRESS_QUEUE_SIZE 32U

static volatile uint32_t pressQueue[PRESS_QUEUE_SIZE];
static volatile uint8_t writeIndex = 0;
static volatile uint8_t readIndex = 0;

static uint32_t lastPressTime = 0;
static uint8_t havePreviousPress = 0;

static uint32_t firstPressTime = 0;
static uint8_t waitingSecondPress = 0;

void Button_Clear(void){
    uint32_t savedMask = __get_PRIMASK();
    __disable_irq();

    readIndex = writeIndex;
    waitingSecondPress = 0;

    __set_PRIMASK(savedMask);
}

void Button_Init(void){
    Button_Clear();
    lastPressTime = 0;
    havePreviousPress = 0;
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
}

void Button_OnInterrupt(uint16_t pin){
    if (pin != USER_BUTTON_PIN){
        return;
    }

    uint32_t now = HAL_GetTick();

    //ignore another press within 50ms
    if (havePreviousPress && (uint32_t)(now - lastPressTime) < DEBOUNCE_MS){
        return;
    }

    havePreviousPress = 1;
    lastPressTime = now;

    uint8_t next = (uint8_t)((writeIndex + 1U) % PRESS_QUEUE_SIZE);

    if (next != readIndex){
        pressQueue[writeIndex] = now;
        writeIndex = next;
    }
}

ButtonEvent Button_GetEvent(uint32_t *pressTime){
    while (1){
        uint32_t savedMask = __get_PRIMASK();
        __disable_irq();

        // no recorded press waiting in queue
        if (readIndex == writeIndex){
            uint32_t now = HAL_GetTick();

            __set_PRIMASK(savedMask);

            // A first press occurred, but no second press
            // arrived within its 500 ms window.
            if (waitingSecondPress && (uint32_t)(now - firstPressTime) >= DOUBLE_PRESS_MS){
                waitingSecondPress = 0;
                *pressTime = firstPressTime;

                return BUTTON_SINGLE;
            }
            return BUTTON_NONE;
        }

        uint32_t nextPressTime = pressQueue[readIndex];

        // This newer press is too late to form a double
        // press with the previous one.
        if (waitingSecondPress && (uint32_t)(nextPressTime - firstPressTime) >= DOUBLE_PRESS_MS)
        {
            __set_PRIMASK(savedMask);

            waitingSecondPress = 0;
            *pressTime = firstPressTime;

            // Leave the newer press queued for the next call.
            return BUTTON_SINGLE;
        }

        // Remove this timestamp from the queue.
        readIndex = (uint8_t)((readIndex + 1U) % PRESS_QUEUE_SIZE);

        __set_PRIMASK(savedMask);

        if (waitingSecondPress){
            // The second press arrived within 500 ms.
            waitingSecondPress = 0;
            *pressTime = nextPressTime;

            return BUTTON_DOUBLE;
        }

        // This is the first press of a new sequence.
        firstPressTime = nextPressTime;
        waitingSecondPress = 1;
    }
}
