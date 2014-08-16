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

//TODO
#include "../peripherals/usart.h"
#include <stdio.h>

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

/* Debugging function: print the values of the status, nonvolatile config.
 * volatile config, enhanced volatile config, and flag registers.
 * This function assumes that SPI bus is already set up. */
static void print_reg(void){
	const uint8_t read_status_cmd = 0x05;
	const uint8_t read_nonvol_conf_cmd = 0xB5;
	const uint8_t read_vol_conf_cmd = 0x85;
	const uint8_t read_enh_conf_cmd = 0x65;
	const uint8_t read_flag_cmd = 0x70;

	uint8_t status, vol_conf, enh_conf, flag;
	uint16_t nonvol_conf;

	ss_low();
	tx_spi(&read_status_cmd, 1);
	while(spi_is_busy());
	rx_spi(&status, 1);
	while(spi_is_busy());
	ss_high();

	ss_low();
	tx_spi(&read_nonvol_conf_cmd, 1);
	while(spi_is_busy());
	rx_spi(&nonvol_conf, 2);
	while(spi_is_busy());
	ss_high();

	ss_low();
	tx_spi(&read_vol_conf_cmd, 1);
	while(spi_is_busy());
	rx_spi(&vol_conf, 1);
	while(spi_is_busy());
	ss_high();

	ss_low();
	tx_spi(&read_enh_conf_cmd, 1);
	while(spi_is_busy());
	rx_spi(&enh_conf, 1);
	while(spi_is_busy());
	ss_high();

	ss_low();
	tx_spi(&read_flag_cmd, 1);
	while(spi_is_busy());
	rx_spi(&flag, 1);
	while(spi_is_busy());
	ss_high();

	iprintf("N25Q512 External Flash Register Values\r\n");
	iprintf("Status: 0x%02X  Nonvolatile Config: 0x%04X\r\n", status, nonvol_conf);
	iprintf("Volatile Config: 0x%02X  Enhanced Config: 0x%02X\r\n", vol_conf, enh_conf);
	iprintf("Flag: 0x%02X\r\n", flag);
	fflush(stdout);
}

/* Get control of the SPI bus (by waiting until it's available) and perform
 * spi setup. */
static void get_spi(void){
	request_spi(request_spi_callback);
	while(!GotSPI){
		/* Wait */
	}
	setup_spi(0, 0, SPI_CR1_BAUDRATE_FPCLK_DIV_16, SPI_CR1_MSBFIRST);
}

/* Wait until an internal chip write operation has completed.
 * This function assumes that SPI bus has already been set up. */
static void wait_chip_busy(void){
	const uint8_t read_status_cmd = 0x05;
	uint8_t status_reg;
	do {
		ss_low();
		tx_spi(&read_status_cmd, 1);
		while(spi_is_busy());
		rx_spi(&status_reg, 1);
		while(spi_is_busy());
		ss_high();
	} while(status_reg & 0x01);
	/* According to the datasheet, the flag register MUST be read to
	 * complete a program/erase command. */
	const uint8_t read_flag_cmd = 0x70;
	const uint8_t clear_flag_cmd = 0x50;
	uint8_t flag_reg;
	ss_low();
	tx_spi(&read_flag_cmd, 1);
	while(spi_is_busy());
	rx_spi(&flag_reg, 1);
	while(spi_is_busy());
	ss_high();

	ss_low();
	tx_spi(&clear_flag_cmd, 1);
	while(spi_is_busy());
	ss_high();
}

/* Enable writing. */
static void write_en(void){
	const uint8_t write_en_cmd = 0x06;
	ss_low();
	tx_spi(&write_en_cmd, 1);
	while(spi_is_busy());
	ss_high();
}
	
/* Setup required pins and check that the memory chip is responding correctly.
 * Returns 0 on success, -1 on error. */
int init_memory(void){
	/* The correct value of the first three bytes of the memory chip's ID. */
	const uint8_t correct_mem_id[] = {0x20, 0xBA, 0x20};

	/* Setup SS pin (net MEMCS_N in schematic). */
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO6);
	ss_high();

	get_spi();

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
		GotSPI = false;
		return -1;
	}
	//TODO
	delay_ms(20); /* Needed for debugging UART to synchronize. */
	//TODO
	iprintf("Got chip ID\r\n");
	print_reg();

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

	//TODO
	iprintf("Software Reset\r\n");
	print_reg();

	//TODO
	write_en();
	iprintf("Write Enable\r\n");
	print_reg();

	/* Set number of dummy clock cycles to 8. */
	const uint8_t write_vol_config_cmd = 0x81;
	const uint8_t vol_config_8_dummy_bits = 0x83;
	write_en();
	ss_low();
	tx_spi(&write_vol_config_cmd, 1);
	while(spi_is_busy());
	tx_spi(&vol_config_8_dummy_bits, 1);
	while(spi_is_busy());
	ss_high();
	//TODO
	iprintf("Set dummy clock cycles to 8\r\n");
	print_reg();

	/* Enter 4-byte address mode. */
	const uint8_t fourbyte_cmd = 0xB7;
	write_en();
	ss_low();
	tx_spi(&fourbyte_cmd, 1);
	while(spi_is_busy());
	ss_high();
	//TODO
	iprintf("Entered 4-byte address mode\r\n");
	print_reg();

	/* TODO: Optimize driver strength and increase SPI speed. */
	release_spi();
	GotSPI = false;
	return 0;

}

/* TODO: See if the chip supports a deep power down mode. (There's a reference
 * to it in the datasheet, but no complete info). */

/* Wait for any internal operations to complete and put the chip into a
 * low-power state. */
void shutdown_memory(void){
	get_spi();
	wait_chip_busy();
	release_spi();
	GotSPI = false;
	/* The suspend mode is in effect whenever SS is high. */
	/* TODO: Ensure SS remains high when the uC is in sleep. */
}

/* Convert a little-endian address into a big-endian sequence of bytes. */
static void convert_addr(uint32_t little_address, uint8_t *big_bytes){
	big_bytes[0] = little_address & 0x000000FF;
	big_bytes[1] = (little_address >> 8) & 0x000000FF;
	big_bytes[2] = (little_address >> 16) & 0x000000FF;
	big_bytes[3] = (little_address >> 24) & 0x000000FF;
}

/* Read data from memory into the provided buffer. The address is in bytes,
 * and need not be aligned to any boundary. The size is in bytes, and need not
 * be a multiple of 2. */
void read_mem(uint32_t address, uint8_t *data, uint32_t size){
	uint8_t cmd[6]; /* 1 cmd byte + 4 addr bytes + 1 dummy byte. */
	cmd[0] = 0x0C; /* 4-byte address STR fast read cmd. */

	if(0 == size || (address + size) > 67108863 || NULL == data){
		return;
	}
	get_spi();
	wait_chip_busy();

	/* The chip cannot cross a die boundary in a single read,
	 * so a such a read must be broken into two. */
	if(   (address < 33554432) /* First address of 2nd die. */
	   && ((address + (size - 1)) >= 33554432)){
		/* Two reads are needed. */
		uint32_t first_size = 33554432 - address;
		uint32_t second_size = size - first_size;
		
		convert_addr(address, &cmd[1]);
		ss_low();
		tx_spi(cmd, 6);
		while(spi_is_busy());
		rx_spi(data, first_size);
		while(spi_is_busy());
		ss_high();

		convert_addr(33554432, &cmd[1]);
		ss_low();
		tx_spi(cmd, 6);
		while(spi_is_busy());
		rx_spi(data + first_size, second_size);
		while(spi_is_busy());
		ss_high();
	} else {
		/* One read will suffice. */
		convert_addr(address, &cmd[1]);
		ss_low();
		tx_spi(cmd, 6);
		while(spi_is_busy());
		rx_spi(data, size);
		while(spi_is_busy());
		ss_high();
	}
	release_spi();
	GotSPI = false;
}	

/* Program a page (256 byte block).
 * WARNING: Programming a page can take up to 5 ms to complete.
 * This function will return quickly and the process
 * happens internally in the chip, but subsequent memory operations will
 * stall until programming is finished. */
void program_page_mem(uint32_t page, const uint8_t *data){
	uint8_t cmd[5];
	cmd[0] = 0x12; /* Four-byte address program cmd. */

	if(NULL == data || page > 262143){
		return;
	}

	get_spi();
	wait_chip_busy();
	write_en();

	convert_addr(page << 8, &cmd[1]);
	ss_low();
	tx_spi(cmd, 5);
	while(spi_is_busy());
	tx_spi(data, 256);
	while(spi_is_busy());
	ss_high();
}

/* Erase the specified sector(s). The start and end sector numbers are an
 * inclusive range; the specified sectors and all in between will be erased.
 * Erasing sets all bits to 1.
 * WARNING: Erasing a sector can take up to 3 *seconds* to complete.
 * This function will return quickly and the process
 * happens internally in the chip, but subsequent memory operations will
 * stall until erasing is finished. */
void erase_sector_mem(uint16_t start, uint16_t end){
	start = end; /* Suppress warnings */
	start = start;
}

/* Erase the specified subsectors(s). The start and end subsector numbers are an
 * inclusive range; the specified subsectors and all in between will be erased.
 * Erasing sets all bits to 1.
 * WARNING: Erasing a subsector can take up to 800 ms to complete.
 * This function will return quickly and the process
 * happens internally in the chip, but subsequent memory operations will
 * stall until erasing is finished. */
void erase_subsector_mem(uint16_t start, uint16_t end){
	start = end; /* Suppress warnings */
	start = start;
}

/* Erase one of the chip's two 32MB dies.
 *   -die is either 0 (lower 32MB) or 1 (upper 32MB)
 * WARNING: Erasing a die can take up to 4 *minutes* to complete.
 * This function will return quickly and the process
 * happens internally in the chip, but subsequent memory operations will
 * stall until erasing is finished. */
void erase_chip_mem(uint8_t die){
	die = die;
}