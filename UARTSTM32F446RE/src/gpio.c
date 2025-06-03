#include <libopencm3/stm32/gpio.h>
#include "core/gpio.h"
#include "core/gpio-pins.h"

void gpio_setup(void)
{
    gpio_clear(LED_PORT, LED_PIN);
    gpio_clear(UART_PORT, TX_PIN);
    gpio_clear(UART_PORT, RX_PIN);
    
    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN); // LED - OUTPUT
    gpio_mode_setup(UART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, TX_PIN);     // TX  - TRANSMISSOR
    gpio_mode_setup(UART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, RX_PIN);     // RX  - RECEPTOR

    gpio_set_af(UART_PORT, GPIO_AF7, TX_PIN); // TX COMO FUNCÃO ALTERNATIVA 7
    gpio_set_af(UART_PORT, GPIO_AF7, RX_PIN); // RX COMO FUNÇÃO ALTERNATIVA 7

    gpio_set_output_options(UART_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, RX_PIN); // SETAR O RX
}