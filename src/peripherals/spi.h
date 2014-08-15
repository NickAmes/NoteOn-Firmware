/* SPI setup and control functions.
 * This file provides a high-level interface to the on-chip SPI3 peripheral.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 */
#ifndef SPI_H
#define SPI_H
#include <libopencm3/stm32/spi.h>
#include <stdint.h>
#include <stdbool.h>

/* The communication protocol required by the nRF8001 is moderately complex,
 * and cannot be handled entirely by the SPI driver. As a result, the SPI driver
 * only arbitrates access to the SPI peripheral between the nRF8001 and external
 * flash drivers. It also provides some high-level functions for doing data
 * transfers. */

/* A note on callbacks: any callback given to an SPI function should be treated
 * as an interrupt. No waiting, no polling, no blocking. */

/* If true, the peripheral is currently in use by a driver. This variable
 * is READ-ONLY. */
extern bool PeripheralInUse;

/* The callback for the driver waiting to use the peripheral. This variable
 * is READ-ONLY. */
extern void (*volatile SpiWaitingCallback)(void);

/* Initialize the SPI peripheral. */
void init_spi(void);

/* Request the SPI peripheral. spi_available_callback must not be NULL.
 * The callback will be called when the peripheral is available.
 * The request queue is only one level deep, so only two peripheral drivers
 * (bluetooth and external flash) may use the SPI bus. Additionally, this
 * function must not be called by the driver that currently
 * controls the peripheral. */
void request_spi(void (*spi_available_callback)(void));

/* The following functions should only be called by a driver that has control
 * over the SPI peripheral. */

/* Release the SPI peripheral. The current transaction should be completely
 * complete (i.e. SS set high), as a waiting request for the peripheral will
 * be started before this function returns. This function should be called
 * whenever the bus can be released, to prevent one driver from hogging the bus. */
void release_spi(void);

/* Unfortunately, the STM32F3 SPI peripheral does not support interrupt-driven
 * communication without waiting for a transfer to complete (why they couldn't
 * include a transfer complete interrupt is beyond me). Drivers should poll
 * spi_is_busy() to determine when their transfer is complete. */

/* Setup the SPI bus.  This function may cause spurious signals on the SPI bus,
 * so it should be called before the transaction starts (i.e. while SS is high).
 *   -CPOL is the clock polarity (0 or 1)
 *   -CPHA is the clock phase (0 or 1)
 *   -baudrate is one of libopencm3's SPI_CR1_BAUDRATE_FPCLK_DIV_X values.
 *    Baudrates are derived from the low-speed peripheral clock APB1.
 *   -firstbit is either SPI_CR1_MSBFIRST or SPI_CR1_LSBFIRST.   */
void setup_spi(uint8_t cpol, uint8_t cpha, uint8_t baudrate, uint8_t firstbit);

/* Evaluates to 1 when the SPI peripheral is busy transferring data. */
#define spi_is_busy() (((SPI_SR(SPI3) & SPI_SR_FTLVL_FIFO_FULL) || (SPI_SR(SPI3) & SPI_SR_BSY))?1:0)

/* Transmit data using DMA. */
void tx_spi(void *data, uint16_t size);

/* Receive data using DMA. */
void rx_spi(void *data, uint16_t size);

/* Simultaneously transmit and receive data using DMA. */
void rxtx_spi(void *rxdata, void *txdata, uint16_t size);

#endif