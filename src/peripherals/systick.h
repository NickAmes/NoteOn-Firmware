/* System time functions.
 * 
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 and newlib projects.                    */
#ifndef SYSTICK_H
#define SYSTICK_H
#include <stdint.h>

/* System Timer. Contains number of milliseconds elapsed since the SysTick
 * timer was started. */
extern volatile uint32_t SystemTime;


/* Setup the SysTick timer with the given number of cycles per period. The SysTick
 * clock is the system clock. The appropriate period depends on the current
 * system clock, but should be set so that the SysTick interrupt fires every
 * millisecond. */
void init_systick(uint32_t period);

#endif