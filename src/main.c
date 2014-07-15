/* Main program.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 project.                                 */
#include <stdio.h>

#include "peripherals/peripherals.h"

/* Setup all peripherals. */
void init_system(void);

int main(void){
	init_system();

	/*while (1) {
		printf("test\n");
		led_on();
		for (int i = 0; i < 500000; i++)
			__asm__("nop");
		led_off();
		for (int i = 0; i < 500000; i++)
			__asm__("nop");
	}*/
// 	while(1){
// 		printf("float: %f\n\r", 1.1 * (float)STK_CSR);
// 		for (int i = 0; i < 50000; i++)
// 			__asm__("nop");
// 	}
}

/* Setup all peripherals. */
void init_system(void){
	/* It appears that the PLL is horrifically inaccurate. */
	//clock_64MHz_hsi();
	init_usart();
	init_led();
	init_systick(7999);
}