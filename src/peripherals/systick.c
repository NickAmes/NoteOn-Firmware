/* System time functions.
 * 
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 and newlib projects.                    */
#include "systick.h"
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include <libopencm3/stm32/gpio.h>

/* Systick ISR - called when systick reaches 0. */
void sys_tick_handler(void)
{
	gpio_toggle(GPIOA, GPIO13);
	//while(1);
}

/* Setup the systick timer. */
void init_systick(uint32_t period){
	/* Set the clock source to the system clock. */
	STK_RVR = period;
	STK_CVR = 0;
	STK_CSR = 7;
}
	