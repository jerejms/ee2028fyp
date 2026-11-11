#ifndef STATE_H
#define STATE_H

#include <stdint.h>

void State_Init(void); // enter standby
void State_Update(void); //process button and run current state
uint8_t State_IsTerminated(void); //as the name follows, kill the current state

#endif
