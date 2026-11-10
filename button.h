#ifndef BUTTON_H
#define BUTTON_H

#include <stdint.h>

typedef enum{
    BUTTON_NONE,
    BUTTON_SINGLE,
    BUTTON_DOUBLE
} ButtonEvent;

void Button_Init(void);
void Button_OnInterrupt(uint16_t pin); //record press when interupt
ButtonEvent Button_GetEvent(uint32_t *pressTime); //none, single, or double press
void Button_Clear(void); //discard any extra presses when changing modes

#endif
