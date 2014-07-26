/* Main program.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 project.                                 */
#include <stdio.h>
#include <stdbool.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/i2c.h>
#include "peripherals/peripherals.h"
#include "board/board.h"

/* Setup all peripherals. */
void init_system(void);

int main(void){
	init_system();
	
	uint8_t data[6];
	uint8_t flag;
	i2c_ticket_t ticket;
	ticket.done_flag = &flag;
	ticket.at_time = 0;
	
	ticket.addr = 0x1D;
	ticket.data = data;

	ticket.rw = I2C_WRITE;
	ticket.reg = 0x20;
	ticket.size = 1;
	data[0] = 0x67;
	//write_i2c(I2C1, 0x1D, 0x20, 1, data); /* Initialize IMU. Accelerometer. */
	add_ticket_i2c(&ticket);
	led_on();
	delay_ms(100);
	while(1){
		read_i2c(I2C1, 0x1D, 0xA8, 6, data);
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

/* Print a debugging welcome message using write_str(). */
static void welcome_message(void){
	delay_ms(20); /* Helps the computer's UART synchronize. */
	write_str("NoteOn Smart Pen Firmware built on ");
	write_str(__TIME__);
	write_str(" ");
	write_str(__DATE__);
	write_str(".\n\r");
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

	welcome_message();
}