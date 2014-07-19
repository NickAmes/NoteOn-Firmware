/* Main program.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 project.                                 */
#include <stdio.h>
#include <libopencm3/cm3/scb.h>
#include "peripherals/peripherals.h"
#include "board/board.h"
#include <libopencm3/stm32/i2c.h>
#include <stdbool.h>

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
	uint8_t data;
	/* REG_MODE register. */
	data = 0b00010001; /* Operating mode, alarms disabled, voltage only. */
	write_i2c(I2C1, 0x70, 0, 1, &data);
	while(1){
		read_i2c(I2C1, 0x70, 19, 1, &data);
		iprintf("I2C: %d\n\r", (int) data);
		//printf("H\n");
		for (int i = 0; i < 1000000; i++)
			__asm__("nop");
	}
}

/* Setup all peripherals. */
void init_system(void){
	/* Misc. important setup tasks. */
	SCB_CPACR |= ((3UL << 10*2)|(3UL << 11*2)); /* Enable FPU */
	SCB_VTOR = 0x08000000; /* Set vector table location. (Makes interrupts work.) */
	SCB_AIRCR = 0x05FA0300; /* Set 16 interrupt group priorities and 0 sub
	                         * priorities. */

	/* Setup peripherals. */
	clock_72MHz_hse();
	init_systick();
	init_usart();
	init_led();
	init_switches();
	init_i2c();
	
}