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
#include <libopencm3/stm32/timer.h>

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

	welcome_message();

	/* Setup board peripheral drivers. */
	init_led();
	init_switches();
	if(1)status |= ERROR_AUXACCEL;
	if(1)status |= ERROR_BLUETOOTH;
	if(init_memory())status |= ERROR_MEMORY;
	if(init_imu())status |= ERROR_IMU;
	if(init_battery())status |= ERROR_BATTERY;
	return status;
}

/* Shutdown all board peripherals.
 * TODO: uC shutdown and button wakeup. */
void shutdown_system(void){
	shutdown_battery();
	shutdown_memory();
	//TODO: uC shutdown and button wakeup.
}