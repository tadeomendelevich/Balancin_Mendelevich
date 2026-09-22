#ifndef MOTORES_TEST_HAL_H
#define MOTORES_TEST_HAL_H

#include <stdint.h>

typedef struct {
    uint32_t autoreload;
    uint32_t compare[2];
    uint8_t running[2];
} TIM_HandleTypeDef;

#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 1U

int HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel);
int HAL_TIM_PWM_Stop(TIM_HandleTypeDef *timer, uint32_t channel);
void MotoresTest_SetCompare(TIM_HandleTypeDef *timer, uint32_t channel,
                            uint32_t compare);

#define __HAL_TIM_SET_COMPARE(timer, channel, compare) \
    MotoresTest_SetCompare((timer), (channel), (compare))
#define __HAL_TIM_GET_AUTORELOAD(timer) ((timer)->autoreload)

#endif
