/* Main program.
 * 
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 project.                                 */
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <stdio.h>

#include "peripherals/clock.h"
#include "peripherals/usart.h"

/* Setup all peripherals. */
void init_system();

int main(void){
	int i;
	init_system();
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);
	char c;
	_read(0, &c, 1);
	while (1) {
		printf("test\n");
		gpio_toggle(GPIOA, GPIO13);
		for (i = 0; i < 1000000; i++)
			__asm__("nop");
	}
}

/* Setup all peripherals. */
void init_system(){
	/* It appears that the PLL is horrifically inaccurate. */
	//clock_64MHz_hsi();
	init_usart();
}