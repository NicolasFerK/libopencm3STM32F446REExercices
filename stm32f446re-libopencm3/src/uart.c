#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>
#include "uart.h"

#define BAUD_RATE (115200)

static bool data_available = false;

void uart_setup(void)
{
    rcc_periph_clock_enable(RCC_USART2);

     /* Setup USART2 parameters. */
     usart_set_baudrate(USART2, BAUD_RATE);
     usart_set_databits(USART2, 8);
     usart_set_stopbits(USART2, USART_STOPBITS_1);
     usart_set_mode(USART2, USART_MODE_TX_RX);
     usart_set_parity(USART2, USART_PARITY_NONE);
     usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
 
     /* Finally enable the USART. */
     usart_enable(USART2);
}
void uart_write(uint16_t* data, const uint32_t length) 
{
    for (uint32_t i = 0; i < length; i++) 
    {
      uart_write_byte(data[i]);
      
    }
}
void uart_write_byte(uint16_t data) 
{
    usart_send_blocking(USART2, (uint16_t)data);
    //USART_DR(USART2) = (uint16_t)data;
}
uint32_t uart_read(uint16_t* data, const uint32_t length)
{
    if(length > 0 && data_available)
    {
        *data = usart_recv_blocking(USART2);
        data_available = false;
        return 1;
    }
    return 0;
}
uint16_t uart_read_byte(void)
{
    return (uint16_t)usart_recv_blocking(USART2);
}
bool uart_data_available(void)
{
    return data_available;
}