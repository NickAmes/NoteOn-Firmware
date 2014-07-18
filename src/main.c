/* Main program.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 project.                                 */
#include <stdio.h>
#include <libopencm3/cm3/scb.h>
#include "peripherals/peripherals.h"
#include "board/board.h"

/* Setup all peripherals. */
void init_system(void);

int main(void){
	init_system();

// 	while (1) {
// 		//printf("test\n");
// 		led_on();
// 		for (int i = 0; i < 500000; i++)
// 			__asm__("nop");
// 		led_off();
// 		for (int i = 0; i < 500000; i++)
// 			__asm__("nop");
// 	}
	while(1){
		printf("Hello, World!  0x%08X \n\r", (int) SystemTime);
		for (int i = 0; i < 100000; i++)
			__asm__("nop");
	}
}

/* Setup all peripherals. */
void init_system(void){
	/* Misc. important setup tasks. */
	SCB_CPACR |= ((3UL << 10*2)|(3UL << 11*2)); /* Enable FPU */
	SCB_VTOR = 0x08000000; /* Set vector table location. (Makes interrupts work.) */

	/* Setup peripherals. */
	init_systick(8000);
	//clock_72MHz_hse();
	init_usart();
	init_led();
	
}