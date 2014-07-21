/* System time functions.
 * 
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 and newlib projects.                    */
#ifndef SYSTICK_H
#define SYSTICK_H
#include <stdint.h>

/* System Timer. Contains number of milliseconds elapsed since the SysTick
 * timer was started.
 * NOTE: This variable will overflow and wrap around after 49.7 days of
 * continuous operation. */
extern volatile uint32_t SystemTime;

/* Setup the SysTick timer so that it fires every millisecond.
 * This function depends on the current system clock. It should be called
 * whenever the system clock changes. */
void init_systick(void);

/* Wait for the given amount of time to elapse before returning. */
void delay_ms(uint32_t milliseconds);

#endif