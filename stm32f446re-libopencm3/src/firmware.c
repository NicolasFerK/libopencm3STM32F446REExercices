#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/scb.h>
#include <stdbool.h>

#define BOOTLOADER_SIZE (0x8000U)

#include "system.h"
#include "timer.h"
#include "uart.h"

#define LED_PORT    (GPIOA)
#define LED_PIN     (GPIO5)

#define UART_PORT   (GPIOA)
#define UART_TX_PIN (GPIO2)
#define UART_RX_PIN (GPIO3)


#define LED_PERIOD (10)
#define INCREASING_CONSTANT .5

static void vector_setup(void)
{
    SCB_VTOR = BOOTLOADER_SIZE;
}

static void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(LED_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, LED_PIN);
    gpio_set_af(LED_PORT, GPIO_AF1, LED_PIN);

    gpio_mode_setup(UART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, UART_TX_PIN | UART_RX_PIN);
    gpio_set_af(UART_PORT, GPIO_AF7, UART_TX_PIN | UART_RX_PIN);
}

int main(void)
{
    vector_setup();
    system_setup();
    gpio_setup();
    timer_setup();
    uart_setup();

    uint64_t startTime = system_get_ticks();
    float duty_cycle = 0.0f;
    bool sentido = 1;

    timer_pwm_set_duty_cycle(duty_cycle);

    while(1)
    {
        if(system_get_ticks()-startTime>=LED_PERIOD)
        {
            //gpio_toggle(LED_PORT, LED_PIN);
            //duty_cycle > 100.0f ? duty_cycle = 0 : duty_cycle+=1;
            if (duty_cycle == 100) sentido = 0;
            else if (duty_cycle == 0) sentido = 1;
                        
            if(sentido==1)
            {
                duty_cycle = (duty_cycle<=100)*(duty_cycle+INCREASING_CONSTANT);
            }
            else
            {
                duty_cycle = (duty_cycle>=0)*(duty_cycle-INCREASING_CONSTANT);
            }
            timer_pwm_set_duty_cycle(duty_cycle);
            startTime = system_get_ticks();
        }
            while (uart_data_available()) 
            {
                uint8_t data = uart_read_byte();
                uart_write_byte(data + 1);
            }

            //system_delay(1000);
    }
    return 0;
}