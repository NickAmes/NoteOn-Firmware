/* I2C Setup and Control Functions
 * This file provides a high-level interface to the I2C1 peripheral.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 project.                              */
#ifndef I2C_H
#define I2C_H

/* If !0, the I2C peripheral is enabled. */
extern int I2CEnabled;

/* Setup the I2C1 peripheral. */
void init_i2c(void);

/* Shutdown the I2C1 peripheral to save power. */
void shutdown_i2c(void);





#endif
