/* SPI setup and control functions.
 * This file provides a high-level interface to the on-chip SPI3 peripheral.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 */
#ifndef SPI_H
#define SPI_H
#include "stdint.h"

/* Priority of SPI interrupts. This is the true numeric value, not the
 * hardware-specific shifted one. */
#define I2C_IRQ_PRIORITY 4

/* The communication protocol required by the nRF8001 is moderately complex,
 * and cannot be handled entirely by the SPI driver. As a result, the SPI driver
 * arbitrates access to the SPI peripheral between the nRF8001 and external
 * flash drivers. It also provides some high-level functions for doing data
 * transfers. */

/* A note on callbacks: any callback given to an SPI function should be treated
 * as an interrupt. No waiting, no polling, no blocking. */

/* Initialize the SPI peripheral. */
void init_spi(void);

/* Request the SPI peripheral. The callback will be called when the
 * peripheral is available. The request queue is only one level deep, so only
 * two peripheral drivers (bluetooth and external flash) may use the SPI bus.
 * Additionally, this function must not be called by the driver that currently
 * controls the peripheral. */
void request_spi(void (*spi_available_callback)(void));

/* The following functions should only be called by a driver that has control
 * over the SPI peripheral. */

/* Release the SPI peripheral. The current transaction should be completely
 * complete (i.e. SS set high), as a waiting request for the peripheral will
 * be started before this function returns. This function should be called
 * whenever the bus can be released, to prevent one driver from hogging the bus. */
void release_spi(void);

/* Setup the SPI bus.
 *   -CPOL is the clock polarity (0 or 1)
 *   -CPHA is the clock phase (0 or 1)
 *   -baudrate is one of libopencm3's SPI_CR1_BAUDRATE_FPCLK_DIV_X values.
 *    Baudrates are derived from the low-speed peripheral clocl APB1.
 *   -lsbfirst - if !0, the least significant bit is transmitted first.
 */
void setup_spi(uint8_t cpol, uint8_t cpha, uint16_t baudrate, bool lsbfirst);

/* Transmit data using DMA.
 * The callback is called when the transmission completes */
void tx_spi(void *data, uint16_t num, void (*tx_done_callback)(void));

/* Receive data using DMA.
 * The callback is called when reception completes */
void rx_spi(void *data, uint16_t num, void (*rx_done_callback)(void));

/* Simultaneously transmit and receive data using DMA.
 * The callback is called when the transfer is complete. */
void rxtx_spi(void *rxdata, void *txdata, uint16_t num,  void (*rxtx_done_callback)(void));

#endif