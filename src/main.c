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

//TODO
#include <libopencm3/stm32/usb.h>

/* Setup all peripherals.
 * The return value indicates the status of the board peripherals. The value
 * is a bitmask in which each peripheral is assigned a bit. If the bit is 0, the
 * peripheral is operating normally. If it is 1, the peripheral
 * has malfunctioned. */
uint8_t init_system(void);

/* init_system() return value bits */
#define ERROR_BATTERY   (1 << 0) /* STC3115 battery gas gauge. */
#define ERROR_IMU       (1 << 1) /* LSM9DS0 IMU. */
#define ERROR_AUXACCEL  (1 << 2) /* LIS3DSH auxiliary accelerometer. */
#define ERROR_BLUETOOTH (1 << 3) /* nRF8001 bluetooth controller. */
#define ERROR_MEMORY    (1 << 4) /* N25Q512A flash memory. */

/* Print a status message describing the state of the board peripherals using
 * init_system()'s return value. */
void print_status_message(uint8_t status);

/* Shutdown all board peripherals.
 * TODO: uC shutdown and button wakeup. */
void shutdown_system(void);

int main(void){
	uint8_t status;
	status = init_system();
	print_status_message(status);

	led_on();
	while(1);

	
// 	uint8_t data[6];
// 	volatile uint8_t flag;
// 	i2c_ticket_t ticket;
// 	ticket.done_flag = &flag;
// 	ticket.at_time = 0;
// 	
// 	ticket.addr = 0x1D;
// 	ticket.data = data;
// 	
// 	ticket.rw = I2C_WRITE;
// 	ticket.reg = 0x20;
// 	ticket.size = 1;
// 	data[0] = 0x67;
// 	add_ticket_i2c(&ticket);
// 
// 	flag = 0;
// 	while(!flag){
// 		/* Wait for transfer to complete before modifying data[0].*/
// 	}
// 	
// 	led_on();
// 	while(1){
// 		data[0] = 0;
// 		data[1] = 0;
// 		data[2] = 0;
// 		data[3] = 0;
// 		data[4] = 0;
// 		data[5] = 0;
// 		ticket.rw = I2C_READ;
// 		ticket.reg = 0xA8;
// 		ticket.size = 6;
// 		flag = 0;
// 		add_ticket_i2c(&ticket);
// 		while(!flag){
// 			/* Wait for data to be available. */
// 			delay_ms(2);
// 			if(flag)break;
// 			iprintf("Waiting for flag\r\n");
// 			delay_ms(100);
// 		}
// 		if(1 == flag){
// 			iprintf("X: %3hd   Y: %3hd   Z: %3hd\n\r", *((int16_t *) &data[0]), *((int16_t *) &data[2]), *((int16_t *) &data[4]));
// 		} else {
// 			iprintf("I2C Error, Flag=%d\n\r", flag);
// 		}
// 		delay_ms(200);
// 		
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

/* Print a status message describing the state of the board peripherals using
 * init_system()'s return value. */
void print_status_message(uint8_t status){
	if(0 == status){
		write_str("All peripherals successfully initialized.\n\r");
	} else {
		write_str("Peripheral Status:\n\r");
		write_str("Gas Gauge            "); write_str((status & ERROR_BATTERY)?"FAIL\r\n":" OK\r\n");
		write_str("IMU                  "); write_str((status & ERROR_IMU)?"FAIL\r\n":" OK\r\n");
		write_str("Aux. Accelerometer   "); write_str((status & ERROR_AUXACCEL)?"FAIL\r\n":" OK\r\n");
		write_str("Bluetooth            "); write_str((status & ERROR_BLUETOOTH)?"FAIL\r\n":" OK\r\n");
		write_str("External Flash       "); write_str((status & ERROR_MEMORY)?"FAIL\r\n":" OK\r\n");
	}
}

/* Setup all peripherals.
 * The return value indicated the status of the board peripherals. The value
 * is a bitmask. Each peripheral is assigned a bit. If the bit is 0, the
 * peripheral is operating normally. If it is 1, the peripheral
 * has malfunctioned. */
uint8_t init_system(void){
	uint8_t status = 0;
	/* Misc. important setup tasks. */
	SCB_CPACR |= ((3UL << 10*2)|(3UL << 11*2)); /* Enable FPU */
	SCB_VTOR = 0x08000000; /* Set vector table location. (Makes interrupts work.) */
	SCB_AIRCR = 0x05FA0300; /* Set 16 interrupt group priorities and 0 sub
	                         * priorities. */

	/* Setup on-chip peripherals. */
	clock_72MHz_hse();
	init_systick();
	init_usart();
	init_i2c();
	init_housekeeping();
	init_spi();
	init_usb();

	/* Setup board peripheral drivers. */
	init_led();
	init_switches();
	if(init_battery())status |= ERROR_BATTERY;
	if(1)status |= ERROR_IMU;
	if(1)status |= ERROR_AUXACCEL;
	if(1)status |= ERROR_BLUETOOTH;
	if(init_memory())status |= ERROR_MEMORY;
	welcome_message();
	return status;
}

/* Shutdown all board peripherals.
 * TODO: uC shutdown and button wakeup. */
void shutdown_system(void){
	shutdown_battery();
	//TODO: uC shutdown and button wakeup.
}