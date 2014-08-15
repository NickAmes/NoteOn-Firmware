/* External flash memory driver.
 * This file implements a high-level interface to a Micron N25Q512A1
 * 512Mb (64MB) SPI NOR flash memory.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 */
#include "memory.h"
#include "../peripherals/spi.h"
#include "../peripherals/systick.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <stdbool.h>
#include <string.h>

/* Bring memory SS pin high. */
#define ss_high() (GPIOA_BSRR |= GPIO6)

/* Bring memory SS pin low. */
#define ss_low() (GPIOA_BSRR |= (GPIO6 << 16))

/* If true, the memory driver has control of the SPI peripheral. */
static bool GotSPI;

/* request_spi() callback. Simply sets GotSPI to true. */
static void request_spi_callback(void){
	GotSPI = true;
}

/* Call request_spi() and wait until the memory driver has control. */
static void get_spi(void){
	request_spi(request_spi_callback);
	while(!GotSPI){
		/* Wait */
	}
}

/* Wait until an internal chip write operation has completed.
 * This function assumes that SPI bus has already been set up. */
static void wait_chip_busy(void){
	const uint8_t read_flag_status_cmd = 0x70;
	uint8_t flag_status_reg;
	do {
		ss_low();
		tx_spi(&read_flag_status_cmd, 1);
		rx_spi(&flag_status_reg, 1);
		ss_high();
	} while(!(flag_status_reg & 0x80));
}
	
	
/* Setup required pins and check that the memory chip is responding correctly.
 * Returns 0 on success, -1 on error. */
int init_memory(void){
	/* The correct value of the first three bytes of the memory chip's ID. */
	const uint8_t correct_mem_id[] = {0x20, 0xBA, 0x20};

	/* Setup SS pin (net MEMCS_N in schematic). */
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);
	ss_high();

	get_spi();
	setup_spi(0, 0, SPI_CR1_BAUDRATE_FPCLK_DIV_16, SPI_CR1_MSBFIRST);

	/* Read chip ID to verify that it's responding. */
	const uint8_t read_id_cmd = 0x9E;
	uint8_t id[3];
	ss_low();
	tx_spi(&read_id_cmd, 1);
	while(spi_is_busy());
	rx_spi(&id, 3);
	while(spi_is_busy());
	ss_high();

	if(0 != memcmp(id, correct_mem_id, 3)){
		/* Incorrect ID. */
		release_spi();
		return -1;
	}

	/* Perform a software reset. */
	const uint8_t reset_enable_cmd = 0x66;
	const uint8_t reset_mem_cmd = 0x99;
	ss_low();
	tx_spi(&reset_enable_cmd, 1);
	while(spi_is_busy());
	ss_high();
	delay_ms(2);
	ss_low();
	tx_spi(&reset_mem_cmd, 1);
	while(spi_is_busy());
	ss_high();
	delay_ms(2);

	release_spi();
	return 0;

}

/* TODO: See if the chip supports a deep power down mode. (There's a reference
 * to it in the datasheet, but no complete info). */

/* Wait for any internal operations to complete and put the chip into a
 * low-power state. */
void shutdown_memory(void);

/* Read data from memory into the provided buffer. The address is in bytes,
 * and need not be aligned to any boundary. The size is in bytes, and need not
 * be a multiple of 2. */
void read_mem(uint32_t address, uint8_t *data, uint32_t size);

/* Program a page (256 byte block).
 * WARNING: Programming a page can take up to 5 ms to complete.
 * This function will return quickly and the process
 * happens internally in the memory, but subsequent memory operations will
 * stall until programming is finished. */
void program_page_mem(uint32_t page, uint8_t *data);

/* Erase the specified sector(s). The start and end sector numbers are an
 * inclusive range; the specified sectors and all in between will be erased.
 * Erasing sets all bits to 1.
 * WARNING: Erasing a sector can take up to 3 *seconds* to complete.
 * This function will return quickly and the process
 * happens internally in the memory, but subsequent memory operations will
 * stall until erasing is finished. */
void erase_sector_mem(uint16_t start, uint16_t end);

/* Erase the specified subsectors(s). The start and end subsector numbers are an
 * inclusive range; the specified subsectors and all in between will be erased.
 * Erasing sets all bits to 1.
 * WARNING: Erasing a subsector can take up to 800 ms to complete.
 * This function will return quickly and the process
 * happens internally in the memory, but subsequent memory operations will
 * stall until erasing is finished. */
void erase_subsector_mem(uint16_t start, uint16_t end);

/* Erase one of the chip's two 32MB dies.
 *   -die is either 0 (lower 32MB) or 1 (upper 32MB)
 *   -passcode must be 0xDEAD to proceed.
 * WARNING: Erasing a die can take up to 4 *minutes* to complete.
 * This function will return quickly and the process
 * happens internally in the memory, but subsequent memory operations will
 * stall until erasing is finished. */
void erase_chip_mem(uint8_t die, uint16_t passcode);