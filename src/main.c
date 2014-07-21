/* Main program.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 project.                                 */
#include <stdio.h>
#include <stdbool.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/i2c.h>
#include "peripherals/peripherals.h"
#include "board/board.h"

#include <libopencm3/cm3/nvic.h>

/* Setup all peripherals. */
void init_system(void);

int main(void){
	init_system();

	uint8_t data[6];
	data[0] = 0x67;
	write_i2c(I2C1, 0x1E, 0x20, 1, data); /* Initialize Aux. Accelerometer. */
	data[0] = 0x10;
	write_i2c(I2C1, 0x1E, 0x25, 1, data); /* Set ADD_INC bit. */
	iprintf("start\n\r"); /* Lets the computer's usart synchronize. */
	delay_ms(100);
	led_on();
	iprintf("Press 's' initiate a self-test routine on the aux. accelerometer.\r\n");
	_read(0, data, 1);
	if('s' == data[0]){
		led_off();
		iprintf("Running self-test...\n\r");
		data[0] = 0x02;  /* Positive sign. */
		write_i2c(I2C1, 0x1E, 0x24, 1, data);
		delay_ms(100);
		data[0] = 0x04;  /* Negative sign. */
		write_i2c(I2C1, 0x1E, 0x24, 1, data);
		delay_ms(100);
		led_on();
		iprintf("Self test complete.\n\r");

	} else {
		iprintf("Self-test canceled.\n\r");
	}
	delay_ms(100);
	while(1){
		read_i2c(I2C1, 0x1E, 0x28, 1, &data[0]);
		read_i2c(I2C1, 0x1E, 0x29, 1, &data[1]);
		read_i2c(I2C1, 0x1E, 0x2A, 1, &data[2]);
		read_i2c(I2C1, 0x1E, 0x2B, 1, &data[3]);
		read_i2c(I2C1, 0x1E, 0x2C, 1, &data[4]);
		read_i2c(I2C1, 0x1E, 0x2D, 1, &data[5]);
		iprintf("X: %3hd   Y: %3hd   Z: %3hd\n\r", *((int16_t *) &data[0]), *((int16_t *) &data[2]), *((int16_t *) &data[4]));
		delay_ms(200);
	}

// 	uint8_t data[2];
// 	uint16_t voltage;
// 	while(1){
// 		data[0] = 0b00010000; /* Soft reset. */
// 		write_i2c(I2C1, 0x70, 1, 1, data);
// 		data[0] = 0b00010001; /* Operating mode, alarms disabled, voltage only. */
// 		write_i2c(I2C1, 0x70, 0, 1, data);
// 		for (int i = 0; i < 10000000; i++)
// 			__asm__("nop");
// 		read_i2c(I2C1, 0x70, 8, 1, &data[0]);
// 		read_i2c(I2C1, 0x70, 9, 1, &data[1]);
// 		voltage = data[0] | (data[1] << 8);
// 		printf("I2C: %f\n\r", (float) voltage * 2.2e-3);
// 	}
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