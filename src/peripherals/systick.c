/* System time functions.
 * 
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 and newlib projects.                    */
#include "systick.h"
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/scb.h>

/* System Timer. Contains number of milliseconds elapsed since the SysTick
 * timer was started.
 * NOTE: This variable will overflow and wrap around after 49.7 days of
 * continuous operation. */
volatile uint32_t SystemTime;

/* Systick ISR - called when SysTick reaches 0. */
void sys_tick_handler(void){
	SystemTime++;
}

/* Setup the SysTick timer with the given number of cycles per period. The SysTick
 * clock is the system clock. The appropriate period depends on the current
 * system clock, but should be set so that the SysTick interrupt fires every
 * millisecond. */
void init_systick(uint32_t period){
	STK_RVR = period - 1; /* Systick counts from 0. */
	STK_CVR = 0;
	/* Enable the SysTick interrupt and set the clock source
	 *to the system clock. */
	STK_CSR = 7;
}
	