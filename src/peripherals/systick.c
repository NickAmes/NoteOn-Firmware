/* System time functions.
 * 
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 and newlib projects.                    */
#include "systick.h"
#include "clock.h"
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

/* Setup the SysTick timer so that it fires every millisecond.
 * This function depends on the current system clock. It should be called
 * whenever the system clock changes. */
void init_systick(void){
	STK_RVR = (SystemClock/1000) - 1; /* Systick counts from 0. */
	STK_CVR = 0;
	/* Enable the SysTick interrupt and set the clock source
	 *to the system clock. */
	STK_CSR = 7;
}

/* Wait for the given amount of time to elapse before returning. */
void delay_ms(uint32_t milliseconds){
	uint32_t target = SystemTime + milliseconds;
	while(SystemTime < target);
}