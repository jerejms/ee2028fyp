#ifndef UART_H
#define UART_H
#include "main.h"

void UART1_Init(void);
HAL_StatusTypeDef UART_SendText(const char *text);

#endif
