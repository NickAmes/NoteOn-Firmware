/* I2C Setup and Control Functions
 * This file provides a high-level interface to the I2C1 peripheral.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 project.                              */
#ifndef I2C_H
#define I2C_H
#include <stdint.h>

/* The I2C system works using a "conveyor" which automatically performs data
 * transfers. Device drivers place "tickets" on the conveyor. The tickets are
 * processed in the order they are received. A flag is set as each ticket is
 * completed. Device drivers poll this flag to know when their data has been
 * transfered. */

/* Ticket structure. */
typedef struct i2c_ticket_t {
	uint8_t rw; /* Transfer direction. 0=write, 1=read. */
	uint8_t addr; /* 7-bit I2C device address. */
	uint8_t reg; /* 8-bit register address. */
	volatile uint8_t *data; /* Data to be transferred - this must not be NULL. */
	uint8_t size; /* Number of bytes to be transferred. */
	volatile uint8_t *done_flag; /* The flag will be set to 1 when the transaction is
	                              * complete, or to 2 if an error occurs. This field
			              * may be NULL. */
	volatile uint32_t *at_time;  /* If not NULL, this will be set the the value of
	                              * SystemTime at the moment the transfer was
			              * completed. If an error occurs, this is not set. */
} i2c_ticket_t;

/* i2c_ticket_t->rw values */
#define I2C_WRITE 0
#define I2C_READ 1

/* i2c_ticket_t->done_flag values */
#define I2C_DONE 1
#define I2C_ERROR 2

/* Number of tickets that the conveyor can hold. */
#define I2C_CONVEYOR_SIZE 8

/* Priority of I2C interrupts. This is the true numeric value, not the
 * hardware-specific shifted one. */
#define I2C_IRQ_PRIORITY 8

/* Add a ticket to the conveyor. The ticket will be copied (and therefore
 * doesn't need to exist after the function call) but the data will not.
 * This function may be called from an interrupt as long as that interrupt has
 * a priority value greater than (less urgent) or equal to I2C_IRQ_PRIORITY.
 * Returns:
 *   0 - Success.
 *  -1 - NULL data field or ticket pointer.
 *  -2 - Conveyor is full. Please try again later. */
int add_ticket_i2c(i2c_ticket_t *ticket);

/* Setup the I2C1 peripheral. */
void init_i2c(void);

/* Shutdown the I2C1 peripheral to save power. */
void shutdown_i2c(void);

#endif
