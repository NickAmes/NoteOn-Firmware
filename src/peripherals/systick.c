/* System time functions.
 * 
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 and newlib projects.                    */
#include "systick.h"
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

/* Systick ISR - called when systick reaches 0. */
void sys_tick_handler(void)
{
	//gpio_toggle(GPIOA, GPIO13);
	while(1);
}

/* Setup the systick timer. */
void init_systick(void){
	/* Set the clock source to the system clock / 8. */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	systick_set_reload(1000000);
	systick_interrupt_enable();
	nvic_set_priority(NVIC_SYSTICK_IRQ, 0);
	systick_counter_enable();
}
	