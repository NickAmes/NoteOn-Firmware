/* System clock setup functions.
 * 
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * NoteOn Contains code from the libopencm3 and newlib projects.              */
#ifndef CLOCK_H
#define CLOCK_H
#include <stdint.h>

/* Current system clock frequency, in Hz. Libopencm3 doesn't seem to have a
 * variable for this. */
extern volatile uint32_t SystemClock;

/* Set the system clock for 72Mhz derived from an external 16Mhz crystal.
 * The APB high-speed clock will be set to 72Mhz, the APB low-speed clock
 * to 36Mhz, and the USB clock to 48Mhz. */
void clock_72MHz_hse(void);

/* Set the system clock for 64Mhz derived from the internal 8MHz oscillator.
 * The APB high-speed clock will be set to 64Mhz and the APB low-speed clock
 * to 32Mhz. */
void clock_64MHz_hsi(void);

#endif