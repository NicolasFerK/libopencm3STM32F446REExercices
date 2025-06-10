#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

#include "includes/uart.h"
#include "includes/ring-buffer.h"
#include "includes/gpio-pins.h"

#define BAUD_RATE (115200)
#define RING_BUFFER_SIZE (128) 

static ring_buffer_t rb = {0U};
static uint8_t data_buffer[RING_BUFFER_SIZE] = {0U};

void usart2_isr(void)
{
	if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) && ((USART_SR(USART2) & USART_SR_RXNE) != 0)) 
    {
        const bool overrun_occurred = usart_get_flag(USART2, USART_FLAG_ORE) == 1;
        const bool received_data = usart_get_flag(USART2, USART_FLAG_RXNE) == 1;

        if(overrun_occurred || received_data)
        {
        if(ring_buffer_write(&rb, (uint8_t)usart_recv(USART2)))
            {
                //Handle Failure?
            }
        }       
	}
}

void usart_setup(void)
{
    ring_buffer_setup(&rb, data_buffer, RING_BUFFER_SIZE);
	/* Enable the USART2 interrupt. */
	nvic_enable_irq(NVIC_USART2_IRQ);

	/* Setup GPIO pins for USART2 transmit. */
	gpio_mode_setup(UART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, TX_PIN);

	/* Setup GPIO pins for USART2 receive. */
	gpio_mode_setup(UART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, RX_PIN);
	gpio_set_output_options(UART_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, RX_PIN);

	/* Setup USART2 TX and RX pin as alternate function. */
	gpio_set_af(UART_PORT, GPIO_AF7, TX_PIN);
	gpio_set_af(UART_PORT, GPIO_AF7, RX_PIN);

	/* Setup USART2 parameters. */
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	/* Enable USART2 Receive interrupt. */
	usart_enable_rx_interrupt(USART2);

	/* Finally enable the USART. */
	usart_enable(USART2);
}

void uart_write(uint8_t* data, const uint32_t length)
{
    for (uint32_t i = 0; i < length; i++) uart_write_byte(data[i]); 
}

void uart_write_byte(uint8_t data)
{ 
    usart_send_blocking(USART2, ((uint16_t)data));
}

uint32_t uart_read(uint8_t* data, const uint32_t length)
{
    if(length == 0) return 0;

    for (uint32_t bytes_read = 0; bytes_read < length; bytes_read++)
    {
        if(!ring_buffer_read(&rb, &data[bytes_read])) return bytes_read;
    }
    return length;
}

uint8_t uart_read_byte(void)
{
    uint8_t byte = 0;

    (void)uart_read(&byte, 1);

    return byte;
}

bool uart_data_available(void)
{
    return !ring_buffer_empty(&rb);
}

void uart_teardown(void)
{
    //USART2
    usart_disable_rx_interrupt(USART2);
    usart_disable(USART2);
    nvic_disable_irq(NVIC_USART2_IRQ);
    rcc_periph_clock_disable(RCC_USART2);
    //GPIO
    gpio_mode_setup(UART_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, TX_PIN | RX_PIN);
    rcc_periph_clock_disable(RCC_GPIOA);    
}
