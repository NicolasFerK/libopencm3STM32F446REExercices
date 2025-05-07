#include <libopencm3/stm32/sdio.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include "core/swd.h"

#define SWDIO_PORT      (GPIOC)
#define SWDIO_PIN       (GPIO0)

#define SWCLOCK_PORT    (GPIOC)
#define SWCLOCK_PIN     (GPIO1)

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
bool swdio_read()
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