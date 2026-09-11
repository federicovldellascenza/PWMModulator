#include "fsm.h"
#include <stdio.h>
#include <string.h>

// Handle delle periferiche (private al modulo)
static TIM_HandleTypeDef *fsm_htim;
static uint32_t fsm_channel;
static ADC_HandleTypeDef *fsm_hadc;
static UART_HandleTypeDef *fsm_huart;

// Stato interno della FSM
static volatile SystemState_t current_state = STATE_MANUAL;
static volatile uint8_t state_changed_flag = 0;

// Variabili per la modalità Auto-Linear
static volatile int32_t duty = 0;
static volatile int32_t direction = 1;

// Utility interna per le stampe UART
static void FSM_Print(const char *str) {
    HAL_UART_Transmit(fsm_huart, (uint8_t*)str, strlen(str), 100);
}

// Inizializzazione della FSM con i reference dell'hardware
void FSM_Init(TIM_HandleTypeDef *htim, uint32_t channel, ADC_HandleTypeDef *hadc, UART_HandleTypeDef *huart) {
    fsm_htim = htim;
    fsm_channel = channel;
    fsm_hadc = hadc;
    fsm_huart = huart;
    current_state = STATE_MANUAL;
}

// Funzione da invocare dentro la callback EXTI
void FSM_HandleButtonPress(void) {
    // Avanza allo stato successivo in modo circolare
    current_state = (SystemState_t)((current_state + 1) % STATE_NUM_STATES);
    state_changed_flag = 1;

    // Reset delle variabili di rampa all'ingresso della modalità lineare
    if (current_state == STATE_LINEAR) {
        duty = 0;
        direction = 1;
    }
}

// Ritorna lo stato corrente
SystemState_t FSM_GetCurrentState(void) {
    return current_state;
}

// Task principale della FSM da invocare nel while(1)
void FSM_Process(void) {
    char msg[64];

    // 1. Notifica cambio stato via UART se scattato l'interrupt
    if (state_changed_flag) {
        state_changed_flag = 0;
        switch (current_state) {
            case STATE_MANUAL: FSM_Print("\r\n>>> MODALITA: MANUALE <<<\r\n"); break;
            case STATE_FREEZE: FSM_Print("\r\n>>> MODALITA: FREEZE <<<\r\n"); break;
            case STATE_LINEAR: FSM_Print("\r\n>>> MODALITA: LINEARE <<<\r\n"); break;
            default: break;
        }
    }

    // 2. Esecuzione della logica specifica dello stato
    switch (current_state) {
        case STATE_MANUAL: {
            HAL_ADC_Start(fsm_hadc);
            if (HAL_ADC_PollForConversion(fsm_hadc, 10) == HAL_OK) {
                uint32_t raw_adc = HAL_ADC_GetValue(fsm_hadc);
                duty = ((uint32_t)raw_adc * 999) / 4095;
                __HAL_TIM_SET_COMPARE(fsm_htim, fsm_channel, (uint32_t)duty);

                snprintf(msg, sizeof(msg), "[MANUAL] Duty: %ld\r\n", duty);
                FSM_Print(msg);
            }
            HAL_ADC_Stop(fsm_hadc);
            HAL_Delay(100);
            break;
        }

        case STATE_FREEZE: {
            // Non aggiorna il TIM COMPARE, mantiene l'ultimo valore impostato
            snprintf(msg, sizeof(msg), "[FREEZE] Duty bloccato: %ld\r\n", duty);
            FSM_Print(msg);
            HAL_Delay(250);
            break;
        }

        case STATE_LINEAR: {
            // Calcolo del prossimo valore della rampa
            int32_t next_duty = duty + (direction * 10);

            if (next_duty >= 999) {
                duty = 999;
                direction = -1;
            } else if (next_duty <= 0) {
                duty = 0;
                direction = 1;
            } else {
                duty = next_duty;
            }

            __HAL_TIM_SET_COMPARE(fsm_htim, fsm_channel, (uint32_t)duty);

            snprintf(msg, sizeof(msg), "[LINEAR] Duty: %ld\r\n", duty);
            FSM_Print(msg);
            HAL_Delay(10);
            break;
        }

        default:
            break;
    }
}