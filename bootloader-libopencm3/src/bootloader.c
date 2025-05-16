//LIBOPENCM3
#include "common-defines.h"
#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
//DEFINES PRÓRPIOS
#include <../../UARTSTM32F446RE/include/core/uart.h>
#include <../../UARTSTM32F446RE/include/core/funcs.h>
#include <../../UARTSTM32F446RE/include/core/system.h>
#include <../../UARTSTM32F446RE/include/core/gpio.h>
#include <comms.h>

#define BOOTLOADER_SIZE        (0x8000)
#define MAIN_APP_START_ADDRESS (FLASH_BASE + BOOTLOADER_SIZE)

static void jump_to_main(void)
{
    typedef void (*void_fn)(void);

    uint32_t* reset_vector_entry = (uint32_t*)(MAIN_APP_START_ADDRESS + 4U); 
    uint32_t* reset_vector = (uint32_t*)(*reset_vector_entry);
    void_fn jump_fn = (void_fn)reset_vector;

    jump_fn();
}

void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(UART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, TX_PIN);     // TX  - TRANSMISSOR
    gpio_mode_setup(UART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, RX_PIN);     // RX  - RECEPTOR

    gpio_set_af(UART_PORT, GPIO_AF7, TX_PIN); // TX COMO FUNCÃO ALTERNATIVA 7
    gpio_set_af(UART_PORT, GPIO_AF7, RX_PIN); // RX COMO FUNÇÃO ALTERNATIVA 7

    gpio_set_output_options(UART_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, RX_PIN); // SETAR O RX
}

int main(void)
{
    system_setup();
    gpio_setup();
    usart_setup();
    comms_setup();

    comms_packet_t packet = 
    {
        .length = 1,
        .data = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
        .crc = 0
    };

    packet.crc = comms_compute_crc(&packet);

    while(true)
    {
        comms_update();
        comms_write(&packet);
        system_delay(500);
    }

    //TEARDOWN


    jump_to_main();

    //NUNCA RETORNA
    return 0;
}