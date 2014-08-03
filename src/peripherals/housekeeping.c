/* Housekeeping service. Periodically performs low-priority tasks like updating
 * the battery voltage. Uses timer 6 to periodically call the housekeeping task.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 and newlib projects.                    */
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include "housekeeping.h"

/* Housekeeping task. */
void tim6_dac_isr(){
	int i;
	for(i=0;i<NUM_HOUSEKEEPING_TASKS;i++){
		if(HouskeepingTasks[i] != 0){
			(HouskeepingTasks[i])();
		}
	}
}

/* Housekeeping task slots. */
void (*volatile HousekeepingTasks[NUM_HOUSEKEEPING_TASKS])(void);

/* Initialize the housekeeping service. This must be called before using any
 * drivers that use the housekeeping service.
 * This function depends on the current system clock. It should be called
 * whenever the system clock changes. */
void init_housekeeping(void){
	/* After enabling timer 6, the prescaler is set to generate a 2kHz
	 * clock. The counter reload register is then set to
	 * ((HOUSEKEEPING_PERIOD_MS<<1) - 1), as the counter counts from zero. */
	rcc_periph_clock_enable(RCC_TIM6);
	