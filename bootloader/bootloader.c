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
#include "includes/aes.h"
//DEFINES PRÓRPIOS

static void clock_setup(void)
{
    //rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_3V3_84MHZ]);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_TIM3);
    rcc_periph_clock_enable(RCC_GPIOC);
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

static const uint8_t secret_key[AES_BLOCK_SIZE] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
};

static void jump_to_main(void)
{
    typedef void (*void_fn)(void);

    uint32_t* reset_vector_entry = (uint32_t*)(MAIN_APP_START_ADDRESS + 4U); 
    uint32_t* reset_vector = (uint32_t*)(*reset_vector_entry);
    void_fn jump_fn = (void_fn)reset_vector;

    jump_fn();
}

static void aes_cbc_mac_step(AES_Block_t aes_state, AES_Block_t prev_state, const AES_Block_t *key_schedule)
{
    //The CBC Chaining Operation
    for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
    {
        ((uint8_t*)aes_state)[i] ^= ((uint8_t*)prev_state)[i];
    }

    AES_EncryptBlock(aes_state, key_schedule);
    memcpy(prev_state, aes_state, AES_BLOCK_SIZE);
    
}

static bool validate_firmware_image(void) 
{
    firmware_info_t* firmware_info = (firmware_info_t*)FWINFO_ADDRESS;
    const uint8_t* signature = (const uint8_t*)SIGNATURE_ADDRESS;

    if(firmware_info->sentinel != FWINFO_SENTINEL)
    {
        return false;
    }

    if(firmware_info->device_id != DEVICE_ID)
    {
        return false;
    }

    AES_Block_t round_keys[NUM_ROUND_KEYS_128];
    AES_KeySchedule128(secret_key, round_keys);

    AES_Block_t aes_state = {0};
    AES_Block_t prev_state = {0};

    uint8_t bytes_to_pad = 16 - (firmware_info->length % 16);

    if(!bytes_to_pad) {
        bytes_to_pad = 16;
    }

    memcpy(aes_state, firmware_info, AES_BLOCK_SIZE);
    aes_cbc_mac_step(aes_state, prev_state, round_keys);

    uint32_t offset = 0;
    while(offset < firmware_info->length)
    {
        //Are we in the point where we need to skip the info and signature sections?
        if(offset == (FWINFO_ADDRESS - MAIN_APP_START_ADDRESS))
        {
            offset += AES_BLOCK_SIZE * 2;
            continue;
        }


        // Are we at the last block?
        if(firmware_info->length - offset > AES_BLOCK_SIZE)
        {
            //The regular case
            memcpy(aes_state, (void*)(MAIN_APP_START_ADDRESS + offset), AES_BLOCK_SIZE);
            aes_cbc_mac_step(aes_state, prev_state, round_keys);
        }
        else
        {
            //The case of padding 
            if(bytes_to_pad == 16)
            {
                // Add a whole extra block of padding
                memcpy(aes_state, (void*)(MAIN_APP_START_ADDRESS + offset), AES_BLOCK_SIZE);
                aes_cbc_mac_step(aes_state, prev_state, round_keys);
                
                memset(aes_state, AES_BLOCK_SIZE, AES_BLOCK_SIZE);
                aes_cbc_mac_step(aes_state, prev_state, round_keys);
            }
            else
            {
                memcpy(aes_state, (void*)(MAIN_APP_START_ADDRESS + offset), AES_BLOCK_SIZE - bytes_to_pad);
                memset((void*)(aes_state) + (AES_BLOCK_SIZE - bytes_to_pad), bytes_to_pad, bytes_to_pad);
            }    
        }
        offset += AES_BLOCK_SIZE;
    }   

    //signature check

    return memcmp(signature, aes_state, AES_BLOCK_SIZE);
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

static void gpio_bootloader_setup(void)
{
    gpio_mode_setup(GPIO_BOOTLOADER_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_BOOTLOADER_PIN);
}

static void gpio_bootloader_teardown(void)
{
    gpio_mode_setup(GPIO_BOOTLOADER_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_BOOTLOADER_PIN);
    rcc_periph_clock_disable(RCC_GPIOC); 
}

int main(void)
{
clock_setup();
gpio_bootloader_setup();
system_setup();
usart_setup();
comms_setup();

simple_timer_setup(&timer1, DEFAULT_TIMEOUT, false); 

    while(stateBL != BL_State_Done && gpio_get(GPIO_BOOTLOADER_PORT, GPIO_BOOTLOADER_PIN))
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
    gpio_bootloader_teardown();
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
/////////////////////////////////////////////////////////////////////////////////////AES///////////////////////////////////////////////////////////////////////////////////
uint8_t GF_Mult(uint8_t a, uint8_t b) {
  uint8_t result = 0;
  uint8_t shiftEscapesField = 0;

  // Loop through byte `b`
  for (uint8_t i = 0; i < 8; i++) {
    // If the LSB is set (i.e. we're not multiplying out by zero for this polynomial term)
    // then we xor the result with `a` (i.e. adding the polynomial terms of a)
    if (b & 1) {
      result ^= a;
    }

    // Double `a`, keeping track of whether that causes `a` to leave the field.
    shiftEscapesField = a & 0x80;
    a <<= 1;

    // Since the next bit we look at in `b` will represent multiplying the terms in `a`
    // by the next power of 2, we can achieve the same result by shifting `a` left.
    // If `a` left the field, we need to modulo with irreduciable polynomial term.
    if (shiftEscapesField) {
      // Note that we use 0x1b instead of 0x11b. If we weren't taking advantage of
      // u8 overflow (i.e. by using u16, we would use the "real" term)
      a ^= 0x1b;
    }

    // Shift `b` down in order to look at the next LSB (worth twice as much in the multiplication)
    b >>= 1;
  }

  return result;
}

void GF_WordAdd(AES_Column_t a, AES_Column_t b, AES_Column_t dest) {
  dest[0] = a[0] ^ b[0];
  dest[1] = a[1] ^ b[1];
  dest[2] = a[2] ^ b[2];
  dest[3] = a[3] ^ b[3];
}

// Spec page 16
const uint8_t sbox_encrypt[] = {
/*          0     1     2     3     4     5     6     7     8     9     a     b     c     d     e     f */
/* 0 */  0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
/* 1 */  0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
/* 2 */  0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
/* 3 */  0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
/* 4 */  0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
/* 5 */  0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
/* 6 */  0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
/* 7 */  0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
/* 8 */  0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
/* 9 */  0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
/* a */  0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
/* b */  0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
/* c */  0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
/* d */  0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
/* e */  0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
/* f */  0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16,
};

// Spec page 22
const uint8_t sbox_decrypt[] = {
/*          0     1     2     3     4     5     6     7     8     9     a     b     c     d     e     f */
/* 0 */  0x52, 0x09, 0x6a, 0xd5, 0x30, 0x36, 0xa5, 0x38, 0xbf, 0x40, 0xa3, 0x9e, 0x81, 0xf3, 0xd7, 0xfb,
/* 1 */  0x7c, 0xe3, 0x39, 0x82, 0x9b, 0x2f, 0xff, 0x87, 0x34, 0x8e, 0x43, 0x44, 0xc4, 0xde, 0xe9, 0xcb,
/* 2 */  0x54, 0x7b, 0x94, 0x32, 0xa6, 0xc2, 0x23, 0x3d, 0xee, 0x4c, 0x95, 0x0b, 0x42, 0xfa, 0xc3, 0x4e,
/* 3 */  0x08, 0x2e, 0xa1, 0x66, 0x28, 0xd9, 0x24, 0xb2, 0x76, 0x5b, 0xa2, 0x49, 0x6d, 0x8b, 0xd1, 0x25,
/* 4 */  0x72, 0xf8, 0xf6, 0x64, 0x86, 0x68, 0x98, 0x16, 0xd4, 0xa4, 0x5c, 0xcc, 0x5d, 0x65, 0xb6, 0x92,
/* 5 */  0x6c, 0x70, 0x48, 0x50, 0xfd, 0xed, 0xb9, 0xda, 0x5e, 0x15, 0x46, 0x57, 0xa7, 0x8d, 0x9d, 0x84,
/* 6 */  0x90, 0xd8, 0xab, 0x00, 0x8c, 0xbc, 0xd3, 0x0a, 0xf7, 0xe4, 0x58, 0x05, 0xb8, 0xb3, 0x45, 0x06,
/* 7 */  0xd0, 0x2c, 0x1e, 0x8f, 0xca, 0x3f, 0x0f, 0x02, 0xc1, 0xaf, 0xbd, 0x03, 0x01, 0x13, 0x8a, 0x6b,
/* 8 */  0x3a, 0x91, 0x11, 0x41, 0x4f, 0x67, 0xdc, 0xea, 0x97, 0xf2, 0xcf, 0xce, 0xf0, 0xb4, 0xe6, 0x73,
/* 9 */  0x96, 0xac, 0x74, 0x22, 0xe7, 0xad, 0x35, 0x85, 0xe2, 0xf9, 0x37, 0xe8, 0x1c, 0x75, 0xdf, 0x6e,
/* a */  0x47, 0xf1, 0x1a, 0x71, 0x1d, 0x29, 0xc5, 0x89, 0x6f, 0xb7, 0x62, 0x0e, 0xaa, 0x18, 0xbe, 0x1b,
/* b */  0xfc, 0x56, 0x3e, 0x4b, 0xc6, 0xd2, 0x79, 0x20, 0x9a, 0xdb, 0xc0, 0xfe, 0x78, 0xcd, 0x5a, 0xf4,
/* c */  0x1f, 0xdd, 0xa8, 0x33, 0x88, 0x07, 0xc7, 0x31, 0xb1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xec, 0x5f,
/* d */  0x60, 0x51, 0x7f, 0xa9, 0x19, 0xb5, 0x4a, 0x0d, 0x2d, 0xe5, 0x7a, 0x9f, 0x93, 0xc9, 0x9c, 0xef,
/* e */  0xa0, 0xe0, 0x3b, 0x4d, 0xae, 0x2a, 0xf5, 0xb0, 0xc8, 0xeb, 0xbb, 0x3c, 0x83, 0x53, 0x99, 0x61,
/* f */  0x17, 0x2b, 0x04, 0x7e, 0xba, 0x77, 0xd6, 0x26, 0xe1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0c, 0x7d,
};

// Spec Appendix A1
AES_Column_t Rcon[] = {
  { 0x01, 0x00, 0x00, 0x00 },
  { 0x02, 0x00, 0x00, 0x00 },
  { 0x04, 0x00, 0x00, 0x00 },
  { 0x08, 0x00, 0x00, 0x00 },
  { 0x10, 0x00, 0x00, 0x00 },
  { 0x20, 0x00, 0x00, 0x00 },
  { 0x40, 0x00, 0x00, 0x00 },
  { 0x80, 0x00, 0x00, 0x00 },
  { 0x1b, 0x00, 0x00, 0x00 },
  { 0x36, 0x00, 0x00, 0x00 },
};

void AES_KeySchedule128(const AES_Key128_t key, AES_Block_t* keysOut) {
  // Track which round key we're on
  AES_Block_t* currentRoundKey = keysOut;

  // The first round key is the key itself
  memcpy(currentRoundKey, key, sizeof(AES_Block_t));

  // Point to the first computed round key
  AES_Block_t* nextRoundKey = currentRoundKey + 1;

  // Temporary copy of the 3rd column
  AES_Column_t col3;
  memcpy(col3, (*currentRoundKey)[3], sizeof(AES_Column_t));

  for (size_t i = 0; i < NUM_ROUND_KEYS_128 - 1; i++) {
    // Modify the last column of the round key
    AES_RotWord(col3);
    AES_SubWord(col3, sbox_encrypt);
    GF_WordAdd(col3, Rcon[i], col3);

    // Compute the next round key
    GF_WordAdd(col3,               (*currentRoundKey)[0], (*nextRoundKey)[0]);
    GF_WordAdd((*nextRoundKey)[0], (*currentRoundKey)[1], (*nextRoundKey)[1]);
    GF_WordAdd((*nextRoundKey)[1], (*currentRoundKey)[2], (*nextRoundKey)[2]);
    GF_WordAdd((*nextRoundKey)[2], (*currentRoundKey)[3], (*nextRoundKey)[3]);

    // Update the last column for the next round
    memcpy(col3, (*nextRoundKey)[3], sizeof(AES_Column_t));

    // Move the current and next round key pointers
    currentRoundKey++;
    nextRoundKey++;
  }
}

void AES_RotWord(AES_Column_t word) {
  uint8_t temp = word[0];
  word[0] = word[1];
  word[1] = word[2];
  word[2] = word[3];
  word[3] = temp;
}

void AES_SubBytes(AES_Block_t state, const uint8_t table[]) {
  uint8_t index;
  for (size_t col = 0; col < 4; col++) {
    for  (size_t row = 0; row < 4; row++) {
      index = state[col][row];
      state[col][row] = table[index];
    }
  }
}

void AES_SubWord(AES_Column_t word, const uint8_t table[]) {
  uint8_t index;
  for (size_t i = 0; i < 4; i++) {
    index = word[i];
    word[i] = table[index];
  }
}

void AES_ShiftRows(AES_Block_t state) {
  uint8_t temp0;
  uint8_t temp1;

  // This implementation is a little awkward because of storing columns
  // in each array of the block instead of rows

  // Shift row 1
  // [0] [1] [2] [3] -> [1] [2] [3] [0]
  temp0 = state[0][1];
  state[0][1] = state[1][1];
  state[1][1] = state[2][1];
  state[2][1] = state[3][1];
  state[3][1] = temp0;

  // Shift row 2
  // [0] [1] [2] [3] -> [2] [3] [0] [1]
  temp0 = state[0][2];
  temp1 = state[1][2];
  state[0][2] = state[2][2];
  state[1][2] = state[3][2];
  state[2][2] = temp0;
  state[3][2] = temp1;

  // Shift row 3
  // [0] [1] [2] [3] -> [3] [0] [1] [2]
  temp0 = state[3][3];
  state[3][3] = state[2][3];
  state[2][3] = state[1][3];
  state[1][3] = state[0][3];
  state[0][3] = temp0;
}

void AES_InvShiftRows(AES_Block_t state) {
  uint8_t temp0;
  uint8_t temp1;

  // Shift row 1
  // [0] [1] [2] [3] -> [3] [0] [1] [2]
  temp0 = state[3][1];
  state[3][1] = state[2][1];
  state[2][1] = state[1][1];
  state[1][1] = state[0][1];
  state[0][1] = temp0;

  // Shift row 2
  // [0] [1] [2] [3] -> [2] [3] [0] [1]
  temp0 = state[0][2];
  temp1 = state[1][2];
  state[0][2] = state[2][2];
  state[1][2] = state[3][2];
  state[2][2] = temp0;
  state[3][2] = temp1;

  // Shift row 3
  // [0] [1] [2] [3] -> [1] [2] [3] [0]
  temp0 = state[0][3];
  state[0][3] = state[1][3];
  state[1][3] = state[2][3];
  state[2][3] = state[3][3];
  state[3][3] = temp0;
}

void AES_MixColumns(AES_Block_t state) {
  AES_Column_t temp = { 0 };

  for (size_t i = 0; i < 4; i++) {
    temp[0] = GF_Mult(0x02, state[i][0]) ^ GF_Mult(0x03, state[i][1]) ^ state[i][2] ^ state[i][3];
    temp[1] = state[i][0] ^ GF_Mult(0x02, state[i][1]) ^ GF_Mult(0x03, state[i][2]) ^ state[i][3];
    temp[2] = state[i][0] ^ state[i][1] ^ GF_Mult(0x02, state[i][2]) ^ GF_Mult(0x03, state[i][3]);
    temp[3] = GF_Mult(0x03, state[i][0]) ^ state[i][1] ^ state[i][2] ^ GF_Mult(0x02, state[i][3]);

    state[i][0] = temp[0]; state[i][1] = temp[1]; state[i][2] = temp[2]; state[i][3] = temp[3];
  }
}

void AES_InvMixColumns(AES_Block_t state) {
  AES_Column_t temp = { 0 };

  for (size_t i = 0; i < 4; i++) {
    temp[0] = GF_Mult(0x0e, state[i][0]) ^ GF_Mult(0x0b, state[i][1]) ^ GF_Mult(0x0d, state[i][2]) ^ GF_Mult(0x09, state[i][3]);
    temp[1] = GF_Mult(0x09, state[i][0]) ^ GF_Mult(0x0e, state[i][1]) ^ GF_Mult(0x0b, state[i][2]) ^ GF_Mult(0x0d, state[i][3]);
    temp[2] = GF_Mult(0x0d, state[i][0]) ^ GF_Mult(0x09, state[i][1]) ^ GF_Mult(0x0e, state[i][2]) ^ GF_Mult(0x0b, state[i][3]);
    temp[3] = GF_Mult(0x0b, state[i][0]) ^ GF_Mult(0x0d, state[i][1]) ^ GF_Mult(0x09, state[i][2]) ^ GF_Mult(0x0e, state[i][3]);

    state[i][0] = temp[0]; state[i][1] = temp[1]; state[i][2] = temp[2]; state[i][3] = temp[3];
  }
}

void AES_AddRoundKey(AES_Block_t state, const AES_Block_t roundKey) {
  for (size_t col = 0; col < 4; col++) {
    for  (size_t row = 0; row < 4; row++) {
      state[col][row] ^= roundKey[col][row];
    }
  }
}

void AES_EncryptBlock(AES_Block_t state, const AES_Block_t* keySchedule) {
  AES_Block_t* roundKey = (AES_Block_t*)keySchedule;

  // Initial round key addition
  AES_AddRoundKey(state, *roundKey++);

  // Note that i starts at 1 since the initial round key is applied already
  for (size_t i = 1; i < NUM_ROUND_KEYS_128; i++) {
    AES_SubBytes(state, sbox_encrypt);
    AES_ShiftRows(state);

    // No column mix in the last round. Not that this implementation should be considered for
    // production, but this would constitute a timing based side-channel risk
    if (i < NUM_ROUND_KEYS_128 - 1) {
      AES_MixColumns(state);
    }

    AES_AddRoundKey(state, *roundKey++);
  }
}

void AES_DecryptBlock(AES_Block_t state, const AES_Block_t* keySchedule) {
  AES_Block_t* roundKey = (AES_Block_t*)keySchedule + NUM_ROUND_KEYS_128 - 1;

  // Note that i starts at 1 since the initial round key is applied already
  for (size_t i = 1; i < NUM_ROUND_KEYS_128; i++) {
    AES_AddRoundKey(state, *roundKey--);

    // No column mix in the first round
    if (i != 1) {
      AES_InvMixColumns(state);
    }

    AES_InvShiftRows(state);
    AES_SubBytes(state, sbox_decrypt);
  }

  // Last key addition
  AES_AddRoundKey(state, *roundKey);
}