/* Main program.
 * 
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 project.                                 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <stdio.h>
int main(void){
	int i;
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);
	printf("test");
	while (1) {
		/* Toggle LED. */
		gpio_toggle(GPIOA, GPIO13);
		for (i = 0; i < 3000000; i++) /* Wait a bit. */
			__asm__("nop");
	}
	
}