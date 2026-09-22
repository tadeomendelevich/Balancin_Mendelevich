#ifndef CONTROL_MOTORES_H
#define CONTROL_MOTORES_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Registra TIM3/TIM4, arranca sus cuatro canales PWM y deja duty 0%.
 * Debe llamarse despues de MX_TIM3_Init() y MX_TIM4_Init().
 */
void ControlMotores_Init(TIM_HandleTypeDef *timer_derecho,
                         TIM_HandleTypeDef *timer_izquierdo);

/* Aplica potencia firmada en porcentaje, limitada a [-100, 100].
 * Derecho:  + = TIM3 CH1, - = TIM3 CH2.
 * Izquierdo: + = TIM4 CH2, - = TIM4 CH1 (cableado invertido).
 * Con ambos comandos en cero detiene completamente los cuatro canales.
 */
void ControlMotores_Aplicar(int16_t motor_derecho, int16_t motor_izquierdo);

#endif
