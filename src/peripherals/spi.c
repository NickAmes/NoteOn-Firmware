/* SPI setup and control functions.
 * This file provides a high-level interface to the on-chip SPI3 peripheral.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * NoteOn Contains code from the libopencm3 and newlib projects.              */
#include "spi.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <stdlib.h>

/* If true, the peripheral is currently in use by a driver. */
bool PeripheralInUse;

/* The callback for the driver waiting to use the peripheral. */
void (*volatile SpiWaitingCallback)(void);

/* Initialize the SPI peripheral. */
void init_spi(void){
	rcc_periph_clock_enable(RCC_SPI3);
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3 | GPIO4 | GPIO5);
	gpio_set_af(GPIOB, GPIO_AF6, GPIO3 | GPIO4 | GPIO5);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO3 | GPIO5);
}

/* Request the SPI peripheral. spi_available_callback must not be NULL.
 * The callback will be called when the peripheral is available.
 * The request queue is only one level deep, so only two peripheral drivers
 * (bluetooth and external flash) may use the SPI bus. Additionally, this
 * function must not be called by the driver that currently
 * controls the peripheral. */
void request_spi(void (*spi_available_callback)(void)){
	if(PeripheralInUse){
		SpiWaitingCallback = spi_available_callback;
	} else {
		PeripheralInUse = true;
		spi_available_callback();
	}
}

/* Used to determine if cleanup is needed between DMA transfers. */
static bool DmaCleanupNeeded;
static bool TxSpi;

/* Disable and reset the SPI peripheral using the procedure outlined in the
 * STM32F302 Reference Manual (section 28.5.8 pg. 815) without blocking.
 * This function assumes that a transaction isn't currently in progress. */
static void disable_and_reset_spi_properly(void){
	dma_disable_channel(DMA1, DMA_CHANNEL3);
	dma_disable_channel(DMA1, DMA_CHANNEL2);
	spi_disable(SPI3);
	rcc_periph_reset_pulse(RST_SPI3);
	DmaCleanupNeeded = false;
	TxSpi = false;
}

/* Release the SPI peripheral. The current transaction should be completely
 * complete (i.e. SS set high), as a waiting request for the peripheral will
 * be started before this function returns. This function should be called
 * whenever the bus can be released, to prevent one driver from hogging the bus. */
void release_spi(void){
	disable_and_reset_spi_properly();
	PeripheralInUse = false;
	if(NULL != SpiWaitingCallback){
		PeripheralInUse = true;
		SpiWaitingCallback();
		SpiWaitingCallback = NULL;
	}
}

/* Setup the SPI bus.  This function may cause spurious signals on the SPI bus,
 * so it should be called before the transaction starts (i.e. while SS is high).
 *   -CPOL is the clock polarity (0 or 1)
 *   -CPHA is the clock phase (0 or 1)
 *   -baudrate is one of libopencm3's SPI_CR1_BAUDRATE_FPCLK_DIV_X values.
 *    Baudrates are derived from the low-speed peripheral clock APB1.
 *   -firstbit is either SPI_CR1_MSBFIRST or SPI_CR1_LSBFIRST.   */
void setup_spi(uint8_t cpol, uint8_t cpha, uint8_t baudrate, uint8_t firstbit){
	disable_and_reset_spi_properly();

	SPI_CR1(SPI3) &= 0xFFC7; /* Mask off baudrate bits. */
	SPI_CR1(SPI3) |= baudrate;
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
	SPI_CR1(SPI3) &= ~SPI_CR1_LSBFIRST;
	SPI_CR1(SPI3) |= firstbit;
	spi_enable_software_slave_management(SPI3);
	spi_set_nss_high(SPI3);
	spi_set_master_mode(SPI3);
	spi_set_data_size(SPI3, SPI_CR2_DS_8BIT);
	spi_fifo_reception_threshold_8bit(SPI3);
	SPI_I2SCFGR(SPI3) &= ~SPI_I2SCFGR_I2SMOD;

	/* All DMA configuration is handled by tx_spi(), rx_spi(), and rxtx_spi(). */
}

/* Cleanup between DMA transfers. */
static void cleanup_dma_spi(void){
	/* Disable SPI without resetting the peripheral. */
	dma_disable_channel(DMA1, DMA_CHANNEL3);
	dma_disable_channel(DMA1, DMA_CHANNEL2);
	spi_disable(SPI3);
	if(TxSpi){
		/* After tx_spi() completes there will be several nonsense bytes
		 * in the RXFIFO. They must be cleared out so that they don't
		 * corrupt a subsequent SPI read. */
		uint8_t throwaway;
		throwaway = SPI_DR(SPI3);
		throwaway = SPI_DR(SPI3);
		throwaway = SPI_DR(SPI3);
		throwaway = SPI_DR(SPI3);
		throwaway = throwaway; /* Suppress compiler warnings. */
		TxSpi = false;
	}
	spi_disable_tx_dma(SPI3);
	spi_disable_rx_dma(SPI3);
	DmaCleanupNeeded = false;
}

/* Transmit data using DMA. */
void tx_spi(const void *data, uint16_t size){
	if(DmaCleanupNeeded)cleanup_dma_spi();
	
	dma_channel_reset(DMA1, DMA_CHANNEL3);
	dma_disable_channel(DMA1, DMA_CHANNEL3);
	dma_set_peripheral_address(DMA1, DMA_CHANNEL3, (uint32_t) &(SPI_DR(SPI3)));
	dma_set_memory_address(DMA1, DMA_CHANNEL3, (uint32_t) data);
	dma_set_number_of_data(DMA1, DMA_CHANNEL3, size);
	dma_set_priority(DMA1, DMA_CHANNEL3, DMA_CCR_PL_HIGH);
	dma_set_memory_size(DMA1, DMA_CHANNEL3, DMA_CCR_MSIZE_8BIT);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL3, DMA_CCR_PSIZE_8BIT);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL3);
	dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL3);
	dma_set_read_from_memory(DMA1, DMA_CHANNEL3);
	dma_enable_channel(DMA1, DMA_CHANNEL3);

	spi_enable_tx_dma(SPI3);
	spi_enable(SPI3);

	TxSpi = true;
	DmaCleanupNeeded = true;
}

/* Receive data using DMA. */
void rx_spi(void *data, uint16_t size){
	if(DmaCleanupNeeded)cleanup_dma_spi();

	/* This byte is sent continuously when rx_spi() receives data. */
	static const uint8_t RxSpiDummyByte = 0x00;

	spi_enable_rx_dma(SPI3);
	
	dma_channel_reset(DMA1, DMA_CHANNEL2);
	dma_disable_channel(DMA1, DMA_CHANNEL2);
	dma_set_peripheral_address(DMA1, DMA_CHANNEL2, (uint32_t) &(SPI_DR(SPI3)));
	dma_set_memory_address(DMA1, DMA_CHANNEL2, (uint32_t) data);
	dma_set_number_of_data(DMA1, DMA_CHANNEL2, size);
	dma_set_priority(DMA1, DMA_CHANNEL2, DMA_CCR_PL_HIGH);
	dma_set_memory_size(DMA1, DMA_CHANNEL2, DMA_CCR_MSIZE_8BIT);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL2, DMA_CCR_PSIZE_8BIT);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL2);
	dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL2);
	dma_set_read_from_peripheral(DMA1, DMA_CHANNEL2);
	dma_enable_channel(DMA1, DMA_CHANNEL2);

	/* The SPI peripheral is driven by transmitted bytes, so dummy bytes
	 * must be sent in order to receive data. This is accomplished with DMA
	 * by simply not incrementing the memory address. */
	dma_channel_reset(DMA1, DMA_CHANNEL3);
	dma_disable_channel(DMA1, DMA_CHANNEL3);
	dma_set_peripheral_address(DMA1, DMA_CHANNEL3, (uint32_t) &(SPI_DR(SPI3)));
	dma_set_memory_address(DMA1, DMA_CHANNEL3, (uint32_t) &RxSpiDummyByte);
	dma_set_number_of_data(DMA1, DMA_CHANNEL3, size);
	dma_set_priority(DMA1, DMA_CHANNEL3, DMA_CCR_PL_HIGH);
	dma_set_memory_size(DMA1, DMA_CHANNEL3, DMA_CCR_MSIZE_8BIT);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL3, DMA_CCR_PSIZE_8BIT);
	dma_disable_memory_increment_mode(DMA1, DMA_CHANNEL3);
	dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL3);
	dma_set_read_from_memory(DMA1, DMA_CHANNEL3);
	dma_enable_channel(DMA1, DMA_CHANNEL3);
	
	spi_enable_tx_dma(SPI3);
	spi_enable(SPI3);
	DmaCleanupNeeded = true;
}

/* Simultaneously transmit and receive data using DMA. */
void rxtx_spi(void *rxdata, const void *txdata, uint16_t size){
	if(DmaCleanupNeeded)cleanup_dma_spi();

	spi_enable_rx_dma(SPI3);
	
	dma_channel_reset(DMA1, DMA_CHANNEL2);
	dma_disable_channel(DMA1, DMA_CHANNEL2);
	dma_set_peripheral_address(DMA1, DMA_CHANNEL2, (uint32_t) &(SPI_DR(SPI3)));
	dma_set_memory_address(DMA1, DMA_CHANNEL2, (uint32_t) rxdata);
	dma_set_number_of_data(DMA1, DMA_CHANNEL2, size);
	dma_set_priority(DMA1, DMA_CHANNEL2, DMA_CCR_PL_HIGH);
	dma_set_memory_size(DMA1, DMA_CHANNEL2, DMA_CCR_MSIZE_8BIT);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL2, DMA_CCR_PSIZE_8BIT);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL2);
	dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL2);
	dma_set_read_from_peripheral(DMA1, DMA_CHANNEL2);
	dma_enable_channel(DMA1, DMA_CHANNEL2);

	dma_channel_reset(DMA1, DMA_CHANNEL3);
	dma_disable_channel(DMA1, DMA_CHANNEL3);
	dma_set_peripheral_address(DMA1, DMA_CHANNEL3, (uint32_t) &(SPI_DR(SPI3)));
	dma_set_memory_address(DMA1, DMA_CHANNEL3, (uint32_t) txdata);
	dma_set_number_of_data(DMA1, DMA_CHANNEL3, size);
	dma_set_priority(DMA1, DMA_CHANNEL3, DMA_CCR_PL_HIGH);
	dma_set_memory_size(DMA1, DMA_CHANNEL3, DMA_CCR_MSIZE_8BIT);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL3, DMA_CCR_PSIZE_8BIT);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL3);
	dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL3);
	dma_set_read_from_memory(DMA1, DMA_CHANNEL3);
	dma_enable_channel(DMA1, DMA_CHANNEL3);
	
	spi_enable_tx_dma(SPI3);
	spi_enable(SPI3);

	DmaCleanupNeeded = true;
}
