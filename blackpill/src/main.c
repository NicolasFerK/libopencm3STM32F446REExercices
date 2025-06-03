#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>

#include <core/funcs.h>
#include <core/gpio.h>
#include <core/uart.h>
#include <core/system.h>

#define BOOTLOADER_SIZE (0x8000U)

// #define LED_PORT (GPIOC)
// #define LED_PIN  (GPIO13)

// static void vector_setup(void)
// {
//     SCB_VTOR = BOOTLOADER_SIZE;
// }

void gpio_setup()
{
    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
}

void clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_3V3_84MHZ]);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_TIM3);
    rcc_periph_clock_enable(RCC_TIM2);
}

int main(void)
{
    // vector_setup();
    clock_setup();
    system_setup();
    gpio_setup();
    timer_setup();
    usart_setup();
    while (1)
    {
            //    gpio_toggle(LED_PORT, LED_PIN);
        __asm__("nop");
        
        while(uart_data_available())
        {
            uint8_t data = uart_read_byte();
            uart_write_byte(data+1);
        }
    }
    
    return 0;
}


