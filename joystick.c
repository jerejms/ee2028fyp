#include "joystick.h"

#define JOYSTICK_ADDR (0x03 << 1)

HAL_StatusTypeDef Joystick_IsReady(I2C_HandleTypeDef *hi2c){
    return HAL_I2C_IsDeviceReady(hi2c, JOYSTICK_ADDR, 3, 100);
}

HAL_StatusTypeDef Joystick_ReadButtons(I2C_HandleTypeDef *hi2c, uint8_t buttons[5]){
    uint8_t command = 0x01; // GET_DEV_EVENT
    uint8_t response[9];

    // Request the switch events and button states.
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, JOYSTICK_ADDR, &command, 1, 100);
    if (status != HAL_OK){
        return status;
    }
    // Read 4 event bytes followed by 5 button bytes.
    status = HAL_I2C_Master_Receive(hi2c, JOYSTICK_ADDR, response, sizeof(response), 100);
    if (status != HAL_OK){
        return status;
    }
    // Convert active-low readings:
    // buttons[i] = 1 when pressed, 0 when released.
    for (int i = 0; i < 5; i++){
        buttons[i] = (response[4 + i] & 0x01U) ? 0 : 1;
    }
    return HAL_OK;
}
