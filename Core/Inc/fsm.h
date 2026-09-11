#ifndef FSM_H
#define FSM_H

#include "main.h"

// Enumerazione degli stati del sistema
typedef enum {
    STATE_MANUAL = 0,
    STATE_LINEAR = 1,
    STATE_FREEZE = 2,
    STATE_NUM_STATES = 3
} SystemState_t;

// API Pubblica
void FSM_Init(TIM_HandleTypeDef *htim, uint32_t channel, ADC_HandleTypeDef *hadc, UART_HandleTypeDef *huart);
void FSM_Process(void);
void FSM_HandleButtonPress(void);

SystemState_t FSM_GetCurrentState(void);

#endif /* FSM_H */