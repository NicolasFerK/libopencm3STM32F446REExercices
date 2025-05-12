#include "common-defines.h"
#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include "../../stm32f446re-libopencm3/include/uart.h"
#include "../../stm32f446re-libopencm3/include/system.h"
#include "../../stm32f446re-libopencm3/include/simple-timer.h"
#include "comms.h"
#include "bl-flash.h"

#define BOOTLOADER_SIZE        (0x8000)
#define MAIN_APP_START_ADDRESS (FLASH_BASE + BOOTLOADER_SIZE)

#define DEVICE_ID (0x42)
#define SYNC_SEQ_0 (0xC4)
#define SYNC_SEQ_1 (0x55)
#define SYNC_SEQ_2 (0x7E)
#define SYNC_SEQ_3 (0x10)


typedef enum bl_state_t 
{
    BL_State_Sync,
    BL_State_WaitForUpdateReq,
    BL_State_DeviceIDReq,
    BL_State_DeviceIDRes,
    BL_State_FWLengthReq,
    BL_State_FWLengthRes,
    BL_State_EraseApplication,
    BL_State_ReceiveFirmware,
    BL_State_Done,
} bl_state_t;

static bl_state_t state = BL_State_Sync;
static uint32_t fw_length = 0;
static uint32_t bytes_written = 0;
static uint8_t sync_seq[4] = {0};

#define UART_PORT   (GPIOA)
#define UART_TX_PIN (GPIO2)
#define UART_RX_PIN (GPIO3)

static void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(UART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, UART_TX_PIN | UART_RX_PIN);
    gpio_set_af(UART_PORT, GPIO_AF7, UART_TX_PIN | UART_RX_PIN);
}

static void jump_to_main(void)
{
    typedef void (*void_fn)(void);

    uint32_t* reset_vector_entry = (uint32_t*)(MAIN_APP_START_ADDRESS + 4U); 
    uint32_t* reset_vector = (uint32_t*)(*reset_vector_entry);
    void_fn jump_fn = (void_fn)reset_vector;

    jump_fn();
}

int main(void)
{
    system_setup();
    gpio_setup();
    uart_setup();
    comms_setup();
    // FAZER UM TEARDOWN
    simple_timer_t timer;
    comms_packet_t packet;
    
    while (1)
    {
        if(state == BL_State_Sync)
        {
            if(uart_data_available())
            {
                sync_seq[0] = sync_seq[1];
                sync_seq[1] = sync_seq[2];
                sync_seq[2] = sync_seq[3];
                sync_seq[3] = uart_read_byte();

                bool is_match = sync_seq[0] == SYNC_SEQ_0;
                is_match = is_match && sync_seq[1] == SYNC_SEQ_1;
                is_match = is_match && sync_seq[2] == SYNC_SEQ_2;
                is_match = is_match && sync_seq[3] == SYNC_SEQ_3;

                if(is_match)
                {

                }
            }
        }
    }
    
    jump_to_main();

    //NUNCA RETORNA
    return 0;
}