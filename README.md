# STM32 Control System: Multi-Mode Finite State Machine (FSM)

Progetto di un sistema di controllo basato su microcontrollore STM32C031C6 sviluppato su ambiente di simulazione Wokwi.

Il sistema gestisce l'uscita PWM (Timer 3, Canale 2) mediante una Macchina a Stati Finiti (FSM) a tre modalità, commutabili in modo circolare tramite l'interrupt EXTI su pulsante (PA6).

## 🛠️ Architettura e Hardware

- **MCU**: STM32C031C6 (ARM Cortex-M0+)
- **ADC1** (IN0 - PA0): Lettura analogica da potenziometro ($0 - 3.3\text{V}$, risoluzione 12-bit: $0 - 4095$)
- **TIM3** (CH2 - PA7): Generazione segnale PWM (ARR = $999$, periodo $0 - 999$)
- **EXTI** (PA6): Pulsante di cambio stato con debounce a tempo
- **USART2** (PA2/PA3): Interfaccia seriale di debug ($115200\text{ baud}$)

## 🔄 Stati della FSM (SystemState_t)

```
   +-----------------+         Pressione PA6         +-----------------+
   |   STATE_MANUAL  | ----------------------------> |   STATE_FREEZE  |
   +-----------------+                               +-----------------+
            ^                                                 |
            |                 Pressione PA6                   |
            +-------------------------------------------------+
                                    |
                                    v
                             +-----------------+
                             |  STATE_LINEAR   |
                             +-----------------+
```

### 1. STATE_MANUAL (Modalità Manuale)

**Funzionamento**: Inseguimento in tempo reale dell'ADC.

**Logica**: Il valore convertito dall'ADC ($0-4095$) viene rimappato sul periodo del PWM ($0-999$):

$$\text{Duty} = \frac{\text{ADC}_{\text{raw}} \times 999}{4095}$$

**Uscita**: Il registro di compare TIM3->CCR2 viene aggiornato continuamente.

### 2. STATE_FREEZE (Modalità Blocco)

**Funzionamento**: Congelamento del PWM al valore attuale.

**Logica**: L'hardware ignora le variazioni dell'ADC. Le letture analogiche continuano a essere monitorate a terminale seriale per debug, ma il registro TIM3->CCR2 non viene modificato.

### 3. STATE_LINEAR (Modalità Auto-Triangolare)

**Funzionamento**: Generazione automatica di un'onda triangolare (Fade) da $0\%$ a $100\%$ PWM.

**Logica**: Incremento/decremento a passi fissi di $10$ unità. Per prevenire underflow/overflow e glitch di memoria (es. salti anomali a $3549$), l'algoritmo applica un calcolo preventivo con tipo con segno (int32_t) e clipping prima della scrittura sul registro hardware:

```c
int32_t next_duty = duty_linear + (direction * 10);

if (next_duty >= 999) {
    duty_linear = 999;
    direction = -1; // Inverte verso il basso
} else if (next_duty <= 0) {
    duty_linear = 0;
    direction = 1;  // Inverte verso l'alto
} else {
    duty_linear = next_duty;
}
```

## 📂 Struttura del Modulo Refactorizzato

### Core/Inc/fsm.h

```c
#ifndef FSM_H
#define FSM_H

#include "main.h"

/**
 * @brief Enumerazione degli stati operativi della FSM
 */
typedef enum {
    STATE_MANUAL = 0,
    STATE_FREEZE,
    STATE_LINEAR,
    STATE_NUM_STATES
} SystemState_t;

/* API Pubblica */
void FSM_Init(TIM_HandleTypeDef *htim, uint32_t channel, ADC_HandleTypeDef *hadc, UART_HandleTypeDef *huart);
void FSM_Process(void);
void FSM_HandleButtonPress(void);
SystemState_t FSM_GetCurrentState(void);

#endif /* FSM_H */
```

### Core/Src/fsm.c

```c
#include "fsm.h"
#include <stdio.h>
#include <string.h>

/* Handle HW Privati del modulo */
static TIM_HandleTypeDef *fsm_htim;
static uint32_t fsm_channel;
static ADC_HandleTypeDef *fsm_hadc;
static UART_HandleTypeDef *fsm_huart;

/* Stato Interno (Incapsulato) */
static volatile SystemState_t current_state = STATE_MANUAL;
static volatile uint8_t state_changed_flag = 0;

/* Variabili di controllo */
static int32_t duty_linear = 0;
static int32_t direction = 1;
static int32_t duty_manual = 0;

static void FSM_Print(const char *str) {
    HAL_UART_Transmit(fsm_huart, (uint8_t*)str, strlen(str), 100);
}

void FSM_Init(TIM_HandleTypeDef *htim, uint32_t channel, ADC_HandleTypeDef *hadc, UART_HandleTypeDef *huart) {
    fsm_htim = htim;
    fsm_channel = channel;
    fsm_hadc = hadc;
    fsm_huart = huart;
    current_state = STATE_MANUAL;
}

void FSM_HandleButtonPress(void) {
    current_state = (SystemState_t)((current_state + 1) % STATE_NUM_STATES);
    state_changed_flag = 1;

    // Reset della rampa all'ingresso dello stato lineare
    if (current_state == STATE_LINEAR) {
        duty_linear = 0;
        direction = 1;
    }
}

SystemState_t FSM_GetCurrentState(void) {
    return current_state;
}

void FSM_Process(void) {
    char msg[64];

    if (state_changed_flag) {
        state_changed_flag = 0;
        switch (current_state) {
            case STATE_MANUAL: FSM_Print("\r\n>>> MODALITA: MANUALE <<<\r\n"); break;
            case STATE_FREEZE: FSM_Print("\r\n>>> MODALITA: FREEZE <<<\r\n"); break;
            case STATE_LINEAR: FSM_Print("\r\n>>> MODALITA: LINEARE <<<\r\n"); break;
            default: break;
        }
    }

    switch (current_state) {
        case STATE_MANUAL: {
            HAL_ADC_Start(fsm_hadc);
            if (HAL_ADC_PollForConversion(fsm_hadc, 10) == HAL_OK) {
                uint32_t raw_adc = HAL_ADC_GetValue(fsm_hadc);
                duty_manual = ((uint32_t)raw_adc * 999) / 4095;
                __HAL_TIM_SET_COMPARE(fsm_htim, fsm_channel, (uint32_t)duty_manual);

                snprintf(msg, sizeof(msg), "[MANUAL] Duty: %ld\r\n", duty_manual);
                FSM_Print(msg);
            }
            HAL_ADC_Stop(fsm_hadc);
            HAL_Delay(100);
            break;
        }

        case STATE_FREEZE: {
            snprintf(msg, sizeof(msg), "[FREEZE] Duty bloccato: %ld\r\n", duty_manual);
            FSM_Print(msg);
            HAL_Delay(250);
            break;
        }

        case STATE_LINEAR: {
            int32_t next_duty = duty_linear + (direction * 10);

            if (next_duty >= 999) {
                duty_linear = 999;
                direction = -1;
            } else if (next_duty <= 0) {
                duty_linear = 0;
                direction = 1;
            } else {
                duty_linear = next_duty;
            }

            __HAL_TIM_SET_COMPARE(fsm_htim, fsm_channel, (uint32_t)duty_linear);

            snprintf(msg, sizeof(msg), "[LINEAR] Duty: %ld\r\n", duty_linear);
            FSM_Print(msg);
            HAL_Delay(10);
            break;
        }

        default: break;
    }
}
```

### Core/Src/main.c (Integrazione)

```c
#include "main.h"
#include "fsm.h"

/* Handle definiti da STM32CubeMX/HAL */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;

int main(void) {
    HAL_Init();
    SystemClock_Config();
    
    /* Inizializzazione Periferiche GPIO, ADC1, TIM3, USART2 */
    // ...

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    /* Inizializzazione Modulo FSM */
    FSM_Init(&htim3, TIM_CHANNEL_2, &hadc1, &huart2);

    while (1) {
        /* Task Principale della FSM */
        FSM_Process();
    }
}

/* Callback di Interrupt EXTI dal pulsante PA6 */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
    static uint32_t last_tick = 0;

    if (GPIO_Pin == GPIO_PIN_6) {
        // Debounce software a 200 ms
        if ((HAL_GetTick() - last_tick) > 200) {
            FSM_HandleButtonPress();
            last_tick = HAL_GetTick();
        }
    }
}
```
