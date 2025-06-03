//LIBOPENCM3
#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/scb.h>
//PROPRIOS HEADERS
#include <string.h>
#include "includes/common-defines.h"
#include "includes/gpio-pins.h"
#include "includes/gpio.h"
#include "includes/uart.h"
#include "includes/system.h"
#include "includes/comms.h"
#include "includes/ring-buffer.h"
#include "includes/crc.h"
#include "includes/comms.h"
#include "includes/bl-flash.h"
#include "includes/simple-timer.h"
#include "includes/firmware-info.h"
//DEFINES PRÓRPIOS

static void clock_setup(void)
{
    //rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_3V3_84MHZ]);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_TIM3);
    //rcc_periph_clock_enable(RCC_TIM2);
}

#define MAIN_APP_SECTOR_START (2)
#define MAIN_APP_SECTOR_END   (7)

#define PACKET_BUFFER_LENGTH   (8)

#define RING_BUFFER_SIZE       (128)

static ring_buffer_t rb =      {0U};
static volatile uint64_t ticks = 0;

static uint8_t data_buffer[RING_BUFFER_SIZE] = {0U};

typedef enum comms_state_t
{
    CommsState_Length,
    CommsState_Data,
    CommsState_CRC,
} comms_state_t;

static comms_state_t state = CommsState_Length;
static uint8_t data_byte_count = 0;

static comms_packet_t temporary_packet = {.length = 0, .data = {0}, .crc = 0};
static comms_packet_t retx_packet = {.length = 0, .data = {0}, .crc = 0};
static comms_packet_t ack_packet = {.length = 0, .data = {0}, .crc = 0};
static comms_packet_t last_transmitted_packet = {.length = 0, .data = {0}, .crc = 0};

static comms_packet_t packet_buffer[PACKET_BUFFER_LENGTH];
static uint32_t packet_read_index = 0;
static uint32_t packet_write_index = 0;
static uint32_t packet_buffer_mask = PACKET_BUFFER_LENGTH - 1;

#define SYNC_SEQ_0 (0xC4)
#define SYNC_SEQ_1 (0x55)
#define SYNC_SEQ_2 (0x7E)
#define SYNC_SEQ_3 (0x10)

#define DEFAULT_TIMEOUT (5000)

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

static bl_state_t stateBL = BL_State_Sync;
static uint32_t fw_length = 0;
static uint32_t bytes_written = 0;
static uint8_t sync_seq[4] = {0};
static simple_timer_t timer1;
static comms_packet_t temp_packet;

static void jump_to_main(void)
{
    typedef void (*void_fn)(void);

    uint32_t* reset_vector_entry = (uint32_t*)(MAIN_APP_START_ADDRESS + 4U); 
    uint32_t* reset_vector = (uint32_t*)(*reset_vector_entry);
    void_fn jump_fn = (void_fn)reset_vector;

    jump_fn();
}

static bool validate_firmware_image(void) 
{
    firmware_info_t* firmware_info = (firmware_info_t*)FWINFO_ADDRESS;

    if(firmware_info->sentinel != FWINFO_SENTINEL)
    {
        return false;
    }

    if(firmware_info->device_id != DEVICE_ID)
    {
        return false;
    }

    const uint8_t* start_address = (const uint8_t*)FWINFO_VALIDATE_FROM; 
    const uint32_t computed_crc = crc32(start_address, FWINFO_VALIDATE_LENGTH(firmware_info->length)); 

    return computed_crc == firmware_info->crc32;
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

static void bootloading_fail(void)
{
    comms_create_single_byte_packet(&temp_packet, BL_PACKET_NACK_DATA0);
    comms_write(&temp_packet);
    stateBL = BL_State_Done;
}

static void check_for_timeout(void)
{
    if(simple_timer_has_elapsed(&timer1))
    {
        bootloading_fail();
    }    
}

static bool is_device_id_packet(const comms_packet_t* packet)
{
    if(packet->length != 2)
    {
        return false;
    }

    if(packet->data[0] != BL_PACKET_DEVICE_ID_RES_DATA0)
    {
        return false;
    }

    for (uint8_t i = 2; i < PACKET_DATA_LENGTH; i++)
    {
        if(packet->data[i] != 0xff)
        {
            return false;
        }
    }
    return true;
}

static bool is_fw_length_packet(const comms_packet_t* packet)
{
    if(packet->length != 5)
    {
        return false;
    }

    if(packet->data[0] != BL_PACKET_FW_LENGTH_RES_DATA0)
    {
        return false;
    }

    for (uint8_t i = 5; i < PACKET_DATA_LENGTH; i++)
    {
        if(packet->data[i] != 0xff)
        {
            return false;
        }
    }
    return true;
}

int main(void)
{
clock_setup();
system_setup();
usart_setup();
comms_setup();



simple_timer_setup(&timer1, DEFAULT_TIMEOUT, false); 

    while(stateBL != BL_State_Done)
    {

        if(stateBL == BL_State_Sync)
        {
            if(uart_data_available())
            {
                sync_seq[0] = sync_seq[1];
                sync_seq[1] = sync_seq[2];
                sync_seq[2] = sync_seq[3];
                sync_seq[3] = uart_read_byte();

                bool is_match = sync_seq[0] == SYNC_SEQ_0;
                is_match = is_match && (sync_seq[1] == SYNC_SEQ_1);
                is_match = is_match && (sync_seq[2] == SYNC_SEQ_2);
                is_match = is_match && (sync_seq[3] == SYNC_SEQ_3);

                if(is_match)
                {
                    comms_create_single_byte_packet(&temp_packet, BL_PACKET_SYNC_OBSERVED_DATA0);
                    comms_write(&temp_packet);
                    simple_timer_reset(&timer1);
                    stateBL = BL_State_WaitForUpdateReq;
                }
                else
                {
                    check_for_timeout();
                }
            }
            else
            {
                check_for_timeout();
            }
            continue;
        }
        comms_update();

        switch (stateBL)
        {
            case BL_State_WaitForUpdateReq:
            {
                if(comms_packets_available())
                {
                    comms_read(&temp_packet);

                    if(comms_is_single_byte_packet(&temp_packet, BL_PACKET_FW_UPDATE_REQ_DATA0))
                    {
                        simple_timer_reset(&timer1);
                        comms_create_single_byte_packet(&temp_packet, BL_PACKET_FW_UPDATE_RES_DATA0);
                        comms_write(&temp_packet);
                        stateBL = BL_State_DeviceIDReq;
                    }
                    else
                    {
                        bootloading_fail();
                    }
                }
                else
                {
                    check_for_timeout();
                }
            } break;
            case BL_State_DeviceIDReq:
            {
                simple_timer_reset(&timer1);
                comms_create_single_byte_packet(&temp_packet, BL_PACKET_DEVICE_ID_REQ_DATA0);
                comms_write(&temp_packet);
                stateBL = BL_State_DeviceIDRes;
            } break;
            case BL_State_DeviceIDRes:
            {
                if(comms_packets_available())
                {
                    comms_read(&temp_packet);

                    if(is_device_id_packet(&temp_packet) && (temp_packet.data[1] == DEVICE_ID))
                    {
                        simple_timer_reset(&timer1);
                        stateBL = BL_State_FWLengthReq;
                    }
                    else
                    {
                        bootloading_fail();
                    }
                }
                else
                {
                    check_for_timeout();
                }
            } break;
            case BL_State_FWLengthReq:
            {
                simple_timer_reset(&timer1);
                comms_create_single_byte_packet(&temp_packet, BL_PACKET_FW_LENGTH_REQ_DATA0);
                comms_write(&temp_packet);
                stateBL = BL_State_FWLengthRes;
            } break;
            case BL_State_FWLengthRes:
            {
                if(comms_packets_available())
                {
                    comms_read(&temp_packet);

                    fw_length = 
                    (
                        (temp_packet.data[1])       |
                        (temp_packet.data[2] << 8)  |
                        (temp_packet.data[3] << 16) |
                        (temp_packet.data[4] << 24) 
                    );

                    if(is_fw_length_packet(&temp_packet) && (fw_length <= MAX_FW_LENGTH))
                    {
                        stateBL = BL_State_EraseApplication;
                    }
                    else
                    {
                        bootloading_fail();
                    }
                }
                else
                {
                    check_for_timeout();
                }
            } break;
            case BL_State_EraseApplication:
            {
                bl_flash_erase_main_application();
                comms_create_single_byte_packet(&temp_packet, BL_PACKET_READY_FOR_DATA_DATA0);
                comms_write(&temp_packet);
                simple_timer_reset(&timer1);
                stateBL = BL_State_ReceiveFirmware;
            } break;
            case BL_State_ReceiveFirmware:
            {
                if(comms_packets_available())
                {
                    comms_read(&temp_packet);
                    
                    const uint8_t packet_length = (temp_packet.length & 0x0f) + 1;
                    bl_flash_write(MAIN_APP_START_ADDRESS + bytes_written, temp_packet.data, packet_length);
                    bytes_written += packet_length;
                    simple_timer_reset(&timer1);

                    if(bytes_written >= fw_length)
                    {
                        comms_create_single_byte_packet(&temp_packet, BL_PACKET_UPDATE_SUCESSFUL_DATA0);
                        comms_write(&temp_packet);
                        stateBL = BL_State_Done;
                    }
                    else
                    {
                        comms_create_single_byte_packet(&temp_packet, BL_PACKET_READY_FOR_DATA_DATA0);
                        comms_write(&temp_packet);
                    }
                }
                else
                {
                    check_for_timeout();
                }
            } break;
            default:
            {
                stateBL = BL_State_Sync;
            } break;
        }
    }

    system_delay(150); //JEITO BURRO DE FAZER COM QUE O PACOTE DE UPDATE SUCESSFUL CHEGUE SEM COM QUE A USART SEJA DESLIGADA ANTES
    uart_teardown();
    system_teardown();
    
    if(validate_firmware_image())
    {
        jump_to_main();
    }
    else
    {
        //reset device
        scb_reset_core();
    }
    //NUNCA RETORNA
    return 0;
}

//////////////////////////////////////////////////////////////////////////////////CRC//////////////////////////////////////////////////////////////////////////////////////

uint8_t crc8(uint8_t* data, uint32_t length)
{
    uint8_t crc = 0;

    for (uint32_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0x07;
            }
            else
            {
                crc <<= 1;
            }
            
        }
    }
    return crc;
    
}

uint32_t crc32(const uint8_t* data, const uint32_t length)
{
    uint8_t byte;
    uint32_t crc = 0xffffffff;
    uint32_t mask;

    for(uint32_t i = 0; i < length; i++) 
    {
        byte = data[i];
        crc = crc ^ byte;

        for(uint8_t j = 0; j < 8; j++)
        {
            mask = -(crc & 1);
            crc = (crc >> 1) ^ (0xedb88320 & mask);
        }
    }

    return ~crc;
}

//////////////////////////////////////////////////////////////////////////////////UART/////////////////////////////////////////////////////////////////////////////////////
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
/////////////////////////////////////////////////////////////////////////////////COMMS/////////////////////////////////////////////////////////////////////////////////////
bool comms_is_single_byte_packet(const comms_packet_t* packet, uint8_t byte)
{
    if(packet->length != 1)
    {
        return false;
    }

    if(packet->data[0] != byte)
    {
        return false;
    }

    for (uint8_t i = 1; i < PACKET_DATA_LENGTH; i++)
    {
        if(packet->data[i] != 0xff)
        {
            return false;
        }
    }
    return true;
    
}

void comms_create_single_byte_packet(comms_packet_t* packet, uint8_t byte)
{
    memset(packet, 0xff, sizeof(comms_packet_t));
    packet->length = 1;
    packet->data[0] = byte;
    packet->crc = comms_compute_crc(packet);
}

void comms_setup(void)
{
    comms_create_single_byte_packet(&retx_packet, PACKET_RETX_DATA0);
    comms_create_single_byte_packet(&ack_packet, PACKET_ACK_DATA0);
}

void comms_update(void)
{
    while (uart_data_available())
    {
        switch (state)
        {
        case CommsState_Length:
        {
            temporary_packet.length = uart_read_byte();
            state = CommsState_Data;
        }
        break;
        case CommsState_Data:
        {
            temporary_packet.data[data_byte_count++] = uart_read_byte();
            if(data_byte_count >= PACKET_DATA_LENGTH)
            {
                data_byte_count = 0;
                state = CommsState_CRC;
            }
        }
        break;
        case CommsState_CRC:
        {
            temporary_packet.crc = uart_read_byte();

            if (temporary_packet.crc != comms_compute_crc(&temporary_packet))
            {
                comms_write(&retx_packet);
                state = CommsState_Length;
                break;
            }
            
            if(comms_is_single_byte_packet(&temporary_packet, PACKET_RETX_DATA0))
            {
                comms_write(&last_transmitted_packet);
                state = CommsState_Length;
                break;
            }

            if(comms_is_single_byte_packet(&temporary_packet, PACKET_ACK_DATA0))
            {
                state = CommsState_Length;
                break; 
            }
            uint32_t next_write_index = (packet_write_index + 1) & packet_buffer_mask;

            if(next_write_index == packet_read_index)
            {
                __asm__("BKPT #0");
            }

            memcpy(&packet_buffer[packet_write_index], &temporary_packet, sizeof(comms_packet_t));
            packet_write_index = next_write_index;
            comms_write(&ack_packet);
            state = CommsState_Length;
        }
        break;
        default:
            state = CommsState_Length;
        break;
        }
    }
    
}

bool comms_packets_available(void)
{
    return packet_read_index != packet_write_index;
} 

void comms_write(comms_packet_t* packet)
{
    uart_write((uint8_t*)packet, PACKET_LENGTH);
    memcpy(&last_transmitted_packet, packet, sizeof(comms_packet_t));
}

void comms_read(comms_packet_t* packet)
{
    memcpy(packet, &packet_buffer[packet_read_index], sizeof(comms_packet_t));    
    packet_read_index = (packet_read_index + 1) & packet_buffer_mask;
}

uint8_t comms_compute_crc(comms_packet_t* packet)
{
    return crc8((uint8_t*)packet, PACKET_LENGTH - PACKET_CRC_BYTES);
}
/////////////////////////////////////////////////////////////////////////////////RING-BUFFER////////////////////////////////////////////////////////////////////////////////
void ring_buffer_setup(ring_buffer_t* ra, uint8_t* buffer, uint32_t size) 
{
  ra->buffer = buffer;
  ra->read_index = 0;
  ra->write_index = 0;
  ra->mask = size - 1;
}

bool ring_buffer_empty(ring_buffer_t* ra) {return ra->read_index == ra->write_index;}

bool ring_buffer_read(ring_buffer_t* ra, uint8_t* byte) 
{
  uint32_t local_read_index = ra->read_index;
  uint32_t local_write_index = ra->write_index;

  if (local_read_index == local_write_index) return false;

  *byte = ra->buffer[local_read_index];
  local_read_index = (local_read_index + 1) & ra->mask;
  ra->read_index = local_read_index;

  return true;
}

bool ring_buffer_write(ring_buffer_t* ra, uint8_t byte) 
{
  uint32_t local_write_index = ra->write_index;
  uint32_t local_read_index = ra->read_index;

  uint32_t next_write_index = (local_write_index + 1) & ra->mask;

  if (next_write_index == local_read_index) return false;

  ra->buffer[local_write_index] = byte;
  ra->write_index = next_write_index;
  return true;
}
//////////////////////////////////////////////////////////////////////////////////BL-FLASH//////////////////////////////////////////////////////////////////////////////////
void bl_flash_erase_main_application(void)
{
    flash_unlock();

    for (uint8_t sector = MAIN_APP_SECTOR_START; sector <= MAIN_APP_SECTOR_END; sector++)
    {
    flash_erase_sector(sector, FLASH_CR_PROGRAM_X32);
    }
    flash_lock();
}

void bl_flash_write(const uint32_t address, const uint8_t* data, const uint32_t length)
{
    flash_unlock();
    flash_program(address, data, length);
    flash_lock();    
}
////////////////////////////////////////////////////////////////////////////////SIMPLE-TIMER////////////////////////////////////////////////////////////////////////////////
void simple_timer_setup(simple_timer_t* timer, uint64_t wait_time, bool auto_reset)
{
    timer->wait_time = wait_time;
    timer->auto_reset = auto_reset;
    timer->target_time = system_get_ticks() + wait_time;
    timer->has_elapsed = false;
}

bool simple_timer_has_elapsed(simple_timer_t* timer)
{
    uint64_t now = system_get_ticks();
    bool has_elapsed = now >= timer->target_time;

    if(timer->has_elapsed) return false;

    if(has_elapsed)
    {
        if(timer->auto_reset)
        {
            uint64_t drift = now - timer->target_time;
            timer->target_time = (now + timer->wait_time) - drift;
        }
        else
        {
            timer->has_elapsed = true;
        }        
    }

    return has_elapsed;
}

void simple_timer_reset(simple_timer_t* timer)
{
    simple_timer_setup(timer, timer->wait_time, timer->auto_reset);
}
///////////////////////////////////////////////////////////////////////////////////SYSTEM///////////////////////////////////////////////////////////////////////////////////
void tim3_isr(void)
{
    if (TIM_SR(TIM3) & TIM_SR_UIF) {
        TIM_SR(TIM3) &= ~TIM_SR_UIF;
        ticks++;
    }
}   

// static void rcc_setup(void)
// {
//     //rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_3V3_84MHZ]);
// }

void tim3_setup(void)
{
    rcc_periph_reset_pulse(RST_TIM3);

    timer_set_prescaler(TIM3, (CPU_FREQ / 1000) - 1);
    timer_set_period(TIM3, 1); 

    timer_enable_irq(TIM3, TIM_DIER_UIE); 
    nvic_enable_irq(NVIC_TIM3_IRQ); 

    timer_enable_counter(TIM3); 
}

uint64_t system_get_ticks(void)
{
    return ticks;
}

void system_setup(void)
{
    //rcc_setup();
    tim3_setup(); 
}

void system_teardown(void)
{
    timer_disable_counter(TIM3);
    timer_disable_irq(TIM3, TIM_DIER_UIE);
    nvic_disable_irq(NVIC_TIM3_IRQ);
    rcc_periph_clock_disable(RCC_TIM3);
}

void system_delay(uint64_t milliseconds)
{
    uint64_t end = system_get_ticks() + milliseconds;
    while (system_get_ticks() < end) {
       
    }
}

firmware_info_t firmware_info = {
    .sentinel   = FWINFO_SENTINEL,
    .device_id  = DEVICE_ID,
    .version    = 0xffffffff,
    .length     = 0xffffffff,
    .reserved0  = 0xffffffff,
    .reserved1  = 0xffffffff,
    .reserved2  = 0xffffffff,
    .reserved3  = 0xffffffff,
    .reserved4  = 0xffffffff,
    .crc32      = 0xffffffff,
};