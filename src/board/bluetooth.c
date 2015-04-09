/* nRF8001 Bluetooth transceiver driver.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * NoteOn Contains code from the libopencm3 and newlib projects.              */
#include "bluetooth.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include "../peripherals/spi.h"

/* Evaluates to the current state of the RDYn line. */
#define RDYn()  ((GPIOB_IDR & GPIO0)?1:0)

/* Set the REQn line high or low. */
#define REQn_high() (GPIOA_BSRR = GPIO7)
#define REQn_low() (GPIOA_BSRR = (GPIO7 << 16))

/* If true, the memory driver has control of the SPI peripheral. */
static bool GotSPI;

/* request_spi() callback. Simply sets GotSPI to true. */
static void request_spi_callback(void){
	GotSPI = true;
}

/* Get control of the SPI bus (by waiting until it's available) and perform
 * spi setup. */
static void get_spi(void){
	request_spi(request_spi_callback);
	while(!GotSPI){
		/* Wait */
	}
	setup_spi(0, 0, SPI_CR1_BAUDRATE_FPCLK_DIV_16, SPI_CR1_LSBFIRST);
}

/* Release the SPI bus and set GotSPI to 0. */
static void release_spi_bt(void){
	GotSPI = false;
	release_spi();
}

/* Initialize the nRF8001 Bluetooth transceiver.
 * Returns 0 on success, -1 on error. */
int init_bt(void){
	gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO0); /* RDYn */
	REQn_high();
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO7); /* REQNn */
	

/* Put the Bluetooth transceiver into a low power state. */
void shutdown_bt(void);
