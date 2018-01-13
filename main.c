/***************************************************************************
 * Artekit AK-CMSIS-DAP JTAG bootloader
 * https://www.artekit.eu/products/debug/ak-cmsis-dap
 *
 * Written by Ruben Meleca
 * Copyright (c) 2018 Artekit Labs
 * https://www.artekit.eu

### main.c

# AK-CMSIS-DAP bootloader is free software: you can redistribute it and/or
# modify it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# AK-CMSIS-DAP bootloader is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with AK-CMSIS-DAP bootloader.
# If not, see <http://www.gnu.org/licenses/>.

***************************************************************************/

#include <string.h>
#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_usart.h"
#include "em_gpio.h"
#include "em_emu.h"
#include "em_msc.h"

#define FW_BASE_ADDR	0x3000
#define FW_FLASH_TOP	0x10000
#define FW_MAX_LEN		(FW_FLASH_TOP - FW_BASE_ADDR)
#define FW_PAGE_SIZE	0x400
#define FW_MAX_PAGES	(FW_MAX_LEN / FW_PAGE_SIZE)
#define FW_CRC_ADDR		0x0FE00000

#define XMODEM_SOH		0x01
#define XMODEM_EOT		0x04
#define XMODEM_ACK		0x06
#define XMODEM_NAK		0x15
#define XMODEM_ETB		0x17
#define XMODEM_CAN		0x18
#define XMODEM_C		0x43
#define XMODEM_TIMEOUT	10000

#define XMODEM_STATE_START	0
#define XMODEM_STATE_C		1
#define XMODEM_STATE_HEADER	2
#define XMODEM_STATE_STORE	3
#define XMODEM_STATE_END	4

static const uint8_t confirm_string[] = "Press any key to start firmware download...";

typedef struct
{
	uint16_t crc;
	uint32_t fw_len;
} FW_CRC;

typedef struct xmodem_packet
{
	uint8_t dummy;			/* for padding */
	uint8_t header;
	uint8_t number;
	uint8_t number_chksum;
	uint8_t data[128];
	uint8_t crc_low;
	uint8_t crc_high;
} XMODEM_PACKET;

typedef void (*jump_func)(void);

static volatile uint32_t tick_count = 0;
static FW_CRC* fw_crc = (FW_CRC*) FW_CRC_ADDR;
static XMODEM_PACKET rx_packet;
static uint8_t next_seq;
static uint32_t curr_addr = FW_BASE_ADDR;

static void delay_ms(uint32_t ms)
{
	uint32_t ticks = tick_count;
	while (tick_count - ticks < ms);
}

void SysTick_Handler(void)
{
	tick_count++;
}

static void uart_init(void)
{
	USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;

	GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 1);
	GPIO_PinModeSet(gpioPortC, 1, gpioModeInput, 0);

	CMU_ClockEnable(cmuClock_USART1, true);
	USART_InitAsync(USART1, &init);
	USART1->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | USART_ROUTE_LOCATION_LOC0;
}

static void uart_deinit(void)
{
	while (!(USART1->STATUS & USART_STATUS_TXC));

	USART_Reset(USART1);
	CMU_ClockEnable(cmuClock_USART1, false);
	GPIO_PinModeSet(gpioPortC, 0, gpioModeDisabled, 0);
	GPIO_PinModeSet(gpioPortC, 1, gpioModeDisabled, 0);
}

static void gpio_init(void)
{
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_HFPER, true);

	/* Boot button, PA0 input with pull-up */
	GPIO_PinModeSet(gpioPortA, 0, gpioModeInputPull, 1);

	/* LED on PB14 */
	GPIO_PinModeSet(gpioPortB, 14, gpioModePushPull, 0);
	GPIO_PinOutSet(gpioPortB, 14);
}

static void gpio_deinit(void)
{
	GPIO_PinModeSet(gpioPortB, 14, gpioModeInput, 0);
	GPIO_PinModeSet(gpioPortA, 0, gpioModeInput, 0);
}

static void flash_write(uint8_t* data, uint32_t len)
{
	/* len must be aligned to 4 bytes */
	if (len & 0x03)
		return;
	
	/* Check if the limit was reached */
	if (curr_addr + len > FW_FLASH_TOP)
		return;

	MSC_WriteWordFast((uint32_t*) curr_addr, data, len);
	curr_addr += len;
	data += len;
}

static void flash_erase_page(uint32_t page_addr)
{
	MSC->ADDRB = page_addr;
	MSC->WRITECMD = MSC_WRITECMD_LADDRIM;
	MSC->WRITECMD = MSC_WRITECMD_ERASEPAGE;
	while (MSC->STATUS & MSC_STATUS_BUSY);
}

static void flash_erase_all(void)
{
	uint32_t page = FW_BASE_ADDR;

	MSC->WRITECTRL |= MSC_WRITECTRL_WREN;

	while (page < FW_FLASH_TOP)
	{
		flash_erase_page(page);
		page += FW_PAGE_SIZE;
	}

	flash_erase_page(FW_CRC_ADDR);

	MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;
}

static void flash_store_crc(FW_CRC* crc)
{
	MSC_WriteWordFast((uint32_t*) FW_CRC_ADDR, crc, sizeof(FW_CRC));
}

static uint16_t crc16(uint8_t* data, uint32_t len)
{
	uint8_t rotate;
	uint32_t crc = 0;

	while (len)
	{
		len--;
		crc = crc ^ (*data++ << 8);
		rotate = 8;
		while (rotate)
		{
			rotate--;
			crc = crc << 1;
			if (crc & 0x10000)
			{
				crc = (crc ^ 0x1021) & 0xFFFF;
			}
		}
	}

	return (uint16_t) crc;
}

static uint16_t get_fw_crc(uint32_t addr, uint32_t len)
{
	uint8_t* ptr = (uint8_t*) addr;
	return crc16(ptr, len);
}

static uint8_t xmodem_verify(XMODEM_PACKET* packet)
{
	uint16_t crc;
	uint16_t calc_crc;

	/* Check packet number */
	if (packet->number + packet->number_chksum != 255)
		return 0;

	if (packet->number != next_seq)
		return 0;

	/* Check CRC */
	crc = (packet->crc_low << 8) | packet->crc_high;
	calc_crc = crc16(packet->data, sizeof(packet->data));

	return (crc == calc_crc);
}

static void print_string(const uint8_t* string)
{
	while (*string)
		USART_Tx(USART1, *string++);
}

static uint8_t xmodem_download(uint8_t confirm)
{
	uint8_t c;
	uint32_t timeout = 0;
	FW_CRC new_crc;
	uint8_t state = 0;
	uint8_t* ptr = (uint8_t*) &rx_packet;
	uint8_t ret = 0;
	uint8_t first_time = 1;
	
	uart_init();
	
	if (confirm)
	{
		print_string(confirm_string);
		USART_Rx(USART1);
	}

	while (1)
	{
		switch (state)
		{
			case XMODEM_STATE_START:
				/* Read any spurious UART data */
				while (USART1->STATUS & USART_STATUS_RXDATAV)
					c = USART1->RXDATA;

				next_seq = 1;
				curr_addr = FW_BASE_ADDR;
				new_crc.fw_len = 0;
				timeout = tick_count;
				state = XMODEM_STATE_C;
				GPIO_PinOutSet(gpioPortB, 14);
				/* no break */

			/* Send XMODEM_C */
			case XMODEM_STATE_C:
				if (USART1->STATUS & USART_STATUS_RXDATAV)
				{
					state = XMODEM_STATE_HEADER;
					break;
				}

				if (tick_count - timeout > 1000)
				{
					timeout = tick_count;
					USART1->TXDATA = XMODEM_C;
					GPIO_PinOutToggle(gpioPortB, 14);
				}
				break;

			/* Wait for header */
			case XMODEM_STATE_HEADER:
				if (USART1->STATUS & USART_STATUS_RXDATAV)
				{
					rx_packet.header = USART1->RXDATA;
					if (rx_packet.header == XMODEM_SOH)
					{
						ptr = (uint8_t*) &rx_packet.header;
						ptr++;
						timeout = tick_count;
						state = XMODEM_STATE_STORE;
					} else if (rx_packet.header == XMODEM_EOT)
					{
						USART_Tx(USART1, XMODEM_ACK);
 						ret = 1;
 						new_crc.crc = get_fw_crc(FW_BASE_ADDR, new_crc.fw_len);
 						flash_store_crc(&new_crc);
 						state = XMODEM_STATE_END;
					} else {
						ret = 0;
						state = XMODEM_STATE_END;
					}
				}
				break;

			/* Read packet */
			case XMODEM_STATE_STORE:
				/* Check for completion */
				if (ptr == (uint8_t*) &rx_packet + sizeof(rx_packet))
				{
					/* Verify and write */
					GPIO_PinOutToggle(gpioPortB, 14);
					if (xmodem_verify(&rx_packet))
					{
						if (first_time)
						{
							MSC_Init();
							flash_erase_all();
							first_time = 0;
						}

						/* Write */
						flash_write(rx_packet.data, sizeof(rx_packet.data));

						next_seq++;
						new_crc.fw_len += sizeof(rx_packet.data);

						/* Send ACK */
						USART_Tx(USART1, XMODEM_ACK);
					} else {
						/* Send NACK */
						USART_Tx(USART1, XMODEM_NAK);
					}

					state = XMODEM_STATE_HEADER;
				} else {
					/* Store in buffer */
					if (USART1->STATUS & USART_STATUS_RXDATAV)
					{
						*ptr = USART1->RXDATA;
						ptr++;
						timeout = tick_count;
					}

					if (tick_count - timeout >= XMODEM_TIMEOUT)
						/* Start all over */
						state = XMODEM_STATE_START;
				}
				break;

			case XMODEM_STATE_END:
				GPIO_PinOutSet(gpioPortB, 14);
				uart_deinit();
				MSC_Deinit();
				return ret;
		}
	}
}

static uint8_t check_fw_crc(void)
{
	uint16_t crc;

	if (fw_crc->fw_len > FW_MAX_LEN)
		return 0;

	crc = get_fw_crc(FW_BASE_ADDR, fw_crc->fw_len);
	return (crc == fw_crc->crc);
}

static uint8_t check_boot_button(void)
{
	return (GPIO_PinInGet(gpioPortA, 0) ? 0 : 1);
}

static void jump_to_app()
{
	uint32_t* sp = (uint32_t*) FW_BASE_ADDR;
	uint32_t* pc = sp + 1;
	jump_func pfunc = (jump_func) *pc;
	
	__set_MSP(*sp);
	__set_PSP(*sp);
	
	SCB->VTOR = (uint32_t)FW_BASE_ADDR;
	
	(pfunc)();
}

int main(void)
{
	uint32_t freq;
	
	/* Chip errata */
	CHIP_Init();

	freq = SystemCoreClockGet();

	SysTick_Config((freq / 1000) - 1);
	gpio_init();

	delay_ms(100);

	/* Check firmware CRC */
	if (!check_fw_crc())
	{
		/* Download without confirmation */
		xmodem_download(0);
	} else if (check_boot_button())
	{
		/* Download with confirmation */		
		xmodem_download(1);
	}

	gpio_deinit();
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk;

	/* Jump to main app */
	jump_to_app();
	while(1);
}
