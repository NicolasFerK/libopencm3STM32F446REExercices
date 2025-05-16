#ifndef INC_FUNCS_H
#define INC_FUNCS_H

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>

#include <common-defines.h>

//static void vector_setup(void);
void clock_setup(void);
void gpio_setup(void);
void timer_setup(void);
void usart_setup(void);

#endif // INC_FUNCS_H