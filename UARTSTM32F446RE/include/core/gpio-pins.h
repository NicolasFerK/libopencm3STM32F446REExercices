#ifndef INC_GPIO_PINS_H
#define  INC_GPIO_PINS_H

#include <libopencm3/stm32/gpio.h>

#define LED_PORT    (GPIOA)
#define LED_PIN     (GPIO5)
#define UART_PORT   (GPIOA)
#define TX_PIN      (GPIO2)
#define RX_PIN      (GPIO3)  

#endif  // INC_GPIO_PINS_H