#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>

#include <core/funcs.h>
#include "common-defines.h"

void usart2_isr(void)
{
	static uint8_t data;

	if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {

		//gpio_toggle(LED_PORT, LED_PIN);

		data = usart_recv(USART2);
        //data += 1;

        usart_enable_tx_interrupt(USART2);
	}

	if (((USART_CR1(USART2) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART2) & USART_SR_TXE) != 0)) {

        usart_send_blocking(USART2, data);

		usart_disable_tx_interrupt(USART2);
	}
}

void usart_setup(void)
{
    nvic_enable_irq(NVIC_USART2_IRQ);
    usart_set_baudrate(USART2, 115200);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

    usart_enable_rx_interrupt(USART2);
    
    usart_enable(USART2);
}

void uart_teardown(void)
{
  usart_disable_rx_interrupt(USART2);
  usart_disable_tx_interrupt(USART2);
  usart_disable(USART2);
  nvic_disable_irq(NVIC_USART2_IRQ);
  rcc_periph_clock_disable(RCC_USART2);    
}
void uart_write(uint8_t* data, const uint32_t length)
{
    
}
void uart_write_byte(uint8_t data)
{
    
}
uint32_t uart_read(uint8_t* data, const uint32_t length)
{
    
}
uint8_t uart_read_byte(void)
{
    
}
bool uart_data_available(void)
{
    
}
