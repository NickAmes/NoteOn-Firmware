/* SPI setup and control functions.
 * This file provides a high-level interface to the on-chip SPI3 peripheral.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 */
#include "spi.h"
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <stdbool.h>
#include <stdlib.h>

/* If true, the peripheral is currently in use by a driver. */
static bool PeripheralInUse;

/* The callback for the driver waiting to use the peripheral. */
static volatile void (*SpiWaitingCallback)(void);

/* Initialize the SPI peripheral. */
void init_spi(void){
	rcc_periph_clock_enable(RCC_SPI3);
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3 | GPIO4 | GPIO5);
	gpio_set_af(GPIOB, GPIO_AF6, GPIO3 | GPIO4 | GPIO5);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO3 | GPIO5);
}

/* Request the SPI peripheral. The callback will be called when the
 * peripheral is available. The request queue is only one level deep, so only
 * two peripheral drivers (bluetooth and external flash) may use the SPI bus.
 * Additionally, this function must not be called by the driver that currently
 * controls the peripheral. */
void request_spi(void (*spi_available_callback)(void)){
	if(PeripheralInUse){
		SpiWaitingCallback = spi_available_callback;
	} else {
		PeripheralInUse = true;
		spi_available_callback();
	}
}

/* The following functions should only be called by a driver that has control
 * over the SPI peripheral. */

/* Release the SPI peripheral. The current transaction should be completely
 * complete (i.e. SS set high), as a waiting request for the peripheral will
 * be started before this function returns. This function should be called
 * whenever the bus can be released, to prevent one driver from hogging the bus. */
void release_spi(void){
	PeripheralInUse = false;
	if(NULL != SpiWaitingCallback){
		PeripheralInUse = true;
		SpiWaitingCallback();
		SpiWaitingCallback = NULL;
	}
}

/* Setup the SPI bus.
 *   -CPOL is the clock polarity (0 or 1)
 *   -CPHA is the clock phase (0 or 1)
 *   -baudrate is one of libopencm3's SPI_CR1_BAUDRATE_FPCLK_DIV_X values.
 *    Baudrates are derived from the low-speed peripheral clocl APB1.
 *   -lsbfirst - if !0, the least significant bit is transmitted first.
 */
void setup_spi(uint8_t cpol, uint8_t cpha, uint16_t baudrate, bool lsbfirst){
	spi_disable(SPI3);
	
	spi_set_baudrate_prescaler(SPI3, baudrate);
	if(0 == cpol){
		spi_set_clock_polarity_0(SPI3);
	} else {
		spi_set_clock_polarity_1(SPI3);
	}
	if(0 == cpha){
		spi_set_clock_phase_0(SPI3);
	} else {
		spi_set_clock_polarity_1(SPI3);
	}
	spi_set_unidirectional_mode(SPI3); /* bidirectional but in 3-wire */
	spi_set_full_duplex_mode(SPI3);
	if(lsbfirst){
		spi_send_lsb_first(SPI3);
	} else {
		spi_send_msb_first(SPI3);
	}
	spi_enable_software_slave_management(SPI3);
	spi_set_master_mode(SPI3);
	spi_set_data_size(SPI3, SPI_CR2_DS_8BIT);
	spi_fifo_reception_threshold_8bit(SPI3);
	SPI_I2SCFGR(SPI3) &= ~SPI_I2SCFGR_I2SMOD;

	/* All DMA configuration is handled by tx_spi(), rx_spi(), and rxtx_spi(). */
}

/* DMA configuration notes:
 *   -Use 8-bit access mode.
 *   -Configure bits in SPI registers as well.
 *   -

/* Transmit data using DMA.
 * The callback is called when the transmission completes */
void tx_spi(void *data, uint16_t num, void (*tx_done_callback)(void));

/* Receive data using DMA.
 * The callback is called when reception completes */
void rx_spi(void *data, uint16_t num, void (*rx_done_callback)(void));

/* Simultaneously transmit and receive data using DMA.
 * The callback is called when the transfer is complete. */
void rxtx_spi(void *rxdata, void *txdata, uint16_t num,  void (*rxtx_done_callback)(void));