#include <libopencm3/stm32/sdio.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include "core/swd.h"

#define SWDIO_PORT      (GPIOC)
#define SWDIO_PIN       (GPIO0)

#define SWCLOCK_PORT    (GPIOC)
#define SWCLOCK_PIN     (GPIO1)

typedef struct _swd_req
{
    uint8_t start : 1;  // This bit is always 1
    uint8_t APnDP : 1;  // This bis it 0 for DPACC and 1 for APACC
    uint8_t RnW : 1;    // This bit is 0 for Write and 1 for Read
    uint8_t A : 2;      // Has different meaning based on if AP is selected or DP.
    uint8_t parity : 1; // This parity check is done on APnDP,RnW,A bits; If no of 1s is even, parity is 0
    uint8_t stop : 1;   // This bit must be 0 for synchronous SWD, which means always.
    uint8_t park : 1;   // This bit must be 1.
} swd_req;

void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_mode_setup(SWDIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SWDIO_PIN);
    gpio_mode_setup(SWCLOCK_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SWCLOCK_PIN);
}
//DEFINI ESTADO SWDIO PC0
void swdio_write(bool high)
{
    if(high)
    {
        gpio_set(SWDIO_PORT, SWDIO_PIN);
    }
    else
    {
        gpio_clear(SWDIO_PORT, SWDIO_PIN);
    }
}
//LE ESTADO SWDIO PC0
bool swdio_read(bool high)
{

}
//ALTERAR ENTRE SAIDA E ENTRADA SWDIO PC0
void swdio_output() {
    gpio_mode_setup(SWDIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SWDIO_PIN);
}

void swdio_input() {
    gpio_mode_setup(SWDIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SWDIO_PIN);
}
//GERA PULSO DE CLOCK
void swclck_pulse()
{
    GPIOC_ODR |= (1<<1);
    __asm__("nop\nnop\nnop\nnop");
    GPIOC_ODR &=~ (1<<1);
}
//ENVIAR BIT PELO SWDIO COM UM PULS O DE CLOCK
void swd_write_bit(bool bit)
{
    swdio_output();
    swdio_write(bit);
    swclck_pulse();
}
//LE UM BIT COM CLOCK
bool swd_read_bit()
{
    bool bit;
    swdio_input();
    swclck_pulse();
    bit = swdio_read();
    return bit;
}
int main(void)
{
    return 0;
}