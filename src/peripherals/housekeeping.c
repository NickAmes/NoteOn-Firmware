/* Housekeeping service. Periodically performs low-priority tasks (such as
 * checking the battery voltage). Uses TIM6.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * NoteOn Contains code from the libopencm3 and newlib projects.              */
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include "housekeeping.h"
#include "clock.h"

/* Housekeeping task. */
void tim6_dac_isr(){
	TIM6_SR = 0; /* Clear the interrupt flag so the ISR will exit. */
	int i;
	for(i=0;i<NUM_HOUSEKEEPING_TASKS;i++){
		if(HousekeepingTasks[i] != 0){
			(HousekeepingTasks[i])();
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
	/* Before enabling timer 6, the prescaler is set to generate a 2kHz
	 * clock. The counter reload register is then set to
	 * ((HOUSEKEEPING_PERIOD_MS * 2) - 1), as the counter counts from zero. */
	rcc_periph_clock_enable(RCC_TIM6);
	TIM6_PSC = SystemClock / 2000;
	TIM6_ARR = ((HOUSEKEEPING_PERIOD_MS * 2) - 1);
	TIM6_DIER = TIM_DIER_UIE; /* Enable update interrupt. */
	nvic_set_priority(NVIC_TIM6_DAC_IRQ, HOUSEKEEPING_IRQ_PRIORITY << 4);
	nvic_enable_irq(NVIC_TIM6_DAC_IRQ);
	TIM6_CR1 = TIM_CR1_CEN; /* Enable timer. */
}