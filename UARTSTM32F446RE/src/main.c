#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
//FUNÇÕES USADAS PRÓPRIAS
#include <core/funcs.h>
#include "common-defines.h"

#define BOOTLOADER_SIZE (0x8000U)

static void vector_setup(void)
{
    SCB_VTOR = BOOTLOADER_SIZE;
}

void clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_3V3_84MHZ]);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_TIM2);
}

int main(void)
{
    vector_setup();
    clock_setup();
    gpio_setup();
    timer_setup();
    usart_setup();

    while(1)
    {
        // gpio_toggle(LED_PORT, LED_PIN);
        for (uint64_t i = 0; i < 2500000; i++)
        {
            __asm__("nop");
        }
        
    }

    return 0;    
}

