#ifndef JOYSTICK_H
#define JOYSTICK_H

#include "main.h"

HAL_StatusTypeDef Joystick_IsReady(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef Joystick_ReadButtons(I2C_HandleTypeDef *hi2c, uint8_t buttons[5]);

#endif
