#ifndef TEST_USBD_CORE_H
#define TEST_USBD_CORE_H
#include "stm32f4xx_hal.h"
#define USBD_STATE_CONFIGURED 3
typedef struct { uint8_t dev_state; } USBD_HandleTypeDef;
#endif
