/* I2C Setup and Control Functions
 * This file provides a high-level interface to the I2C1 peripheral.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 project.                              */
#include "i2c.h"
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

/* If !0, the I2C peripheral is enabled. */
static volatile uint8_t I2CEnabled;

/* Conveyor. */
static volatile i2c_ticket_t Conveyor[I2C_CONVEYOR_SIZE];

/* Add a ticket to the conveyor. The ticket will be copied (and therefore
 * doesn't need to exist after the function call) but the data will not.
 * Returns:
 *   0 - Success.
 *  -1 - NULL data field or ticket pointer.
 *  -2 - Conveyor is full. Please try again later. */
int add_ticket_i2c(i2c_ticket_t *ticket){
	if(0 == ticket)return -1;
	if(0 == ticket->data)return -1;
	if(0 == I2CEnabled)init_i2c();

	
}


/* Setup the I2C1 peripheral. */
void init_i2c(void){
	rcc_periph_clock_enable(RCC_I2C1);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_set_i2c_clock_hsi(I2C1);
	
	i2c_reset(I2C1);
	/* Setup GPIO pins */
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
	gpio_set_af(GPIOB, GPIO_AF4, GPIO6 | GPIO7);
	i2c_peripheral_disable(I2C1);
	i2c_enable_analog_filter(I2C1);
	i2c_set_digital_filter(I2C1, I2C_CR1_DNF_DISABLED);
	/* Setup I2C Fast Mode (400MHz) */
	i2c_set_prescaler(I2C1, 0);
	i2c_set_scl_low_period(I2C1, 0x9);
	i2c_set_scl_high_period(I2C1, 0x3);
	i2c_set_data_hold_time(I2C1, 0x1);
	i2c_set_data_setup_time(I2C1, 0x3);

	i2c_peripheral_enable(I2C1);
	I2CEnabled = 1;
}

/* Shutdown the I2C1 peripheral to save power. */
void shutdown_i2c(void){
	i2c_peripheral_disable(I2C1);
	rcc_periph_clock_disable(RCC_I2C1);
	I2CEnabled = 0;
}