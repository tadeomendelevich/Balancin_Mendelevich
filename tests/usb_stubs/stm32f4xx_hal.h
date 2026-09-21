#ifndef USB_TEST_HAL_H
#define USB_TEST_HAL_H
#include <stdint.h>
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);
#endif
