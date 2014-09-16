/* Main program.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * NoteOn Contains code from the libopencm3 and newlib projects.              */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/i2c.h>
#include "peripherals/peripherals.h"
#include "board/board.h"

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

/* IMU test data acquisition helper functions. */
static void imu_test(void){
	/* Get data from the IMU without recording it to memory. */
	uint32_t count = 0;
	imu_data_t *data;
	printf("Testing...\r\n");
	fflush(stdout);
	
	start_imu();
	while(1){
		while(NULL == (data = get_buf_imu()));
		release_buf_imu();
		count++;
		if((count % 10) == 0)led_toggle();
		if(BufferOverrunIMU || FIFOOverrunIMU || BusErrorIMU || BusTimeoutIMU){
			printf("BufferOverrunIMU: %d  FIFOOverrunIMU: %d  BusErrorIMU: %d  BusTimeoutIMU: %d\r\n",
			       BufferOverrunIMU, FIFOOverrunIMU, BusErrorIMU, BusTimeoutIMU);
			fflush(stdout);
			break;
		}
	}
}

static void imu_erase(void){
	/* Erase Data. */
	printf("#Erasing...\r\n");
	erase_sector_mem(0);
	delay_ms(time_remaining_mem());
	erase_sector_mem(1);
	delay_ms(time_remaining_mem());
	printf("#Done.\r\n");
}

//if count == 0 run forever
static void imu_record(int count){
	/* Record data to external flash. */
	uint8_t buf[512];
	uint32_t page = 0;
	imu_data_t *data;
	printf("#Recording...\r\n");
	fflush(stdout);
	start_imu();
	if(0 == count){
		count = 3000000;
	}
	while(page < count){
		while(NULL == (data = get_buf_imu()));
		memcpy(buf, data, sizeof(imu_data_t));
		release_buf_imu();
		/* Record error indicators. */
		buf[sizeof(imu_data_t) + 0] = BufferOverrunIMU;
		buf[sizeof(imu_data_t) + 1] = FIFOOverrunIMU;
		buf[sizeof(imu_data_t) + 2] = BusErrorIMU;
		buf[sizeof(imu_data_t) + 3] = BusTimeoutIMU;
		/* Last byte of the odd page is set to 0 to indicate that the
		 * pair of pages contains valid data. */
		buf[511] = 0;
		program_page_mem(page, buf);
		program_page_mem(page + 1, buf + 256);
		page += 2;
		if((page % 10) == 0)led_toggle();
	}
}

static void imu_playback(void){
	/* Read out data over USART. */
	uint8_t buf[512];
	imu_data_t *data = (imu_data_t *)buf;
	uint32_t page = 0;
	uint32_t ia_sample = 0;
	uint32_t g_sample = 0;
	uint32_t aa_sample = 0;
	int i;
	/* Output formats:
	 *   Type Code (AA, IA, G, or M), Time (s), X, Y, Z, Temperature, Tip Switch State, Memory Page
	 *  #BufferOverrunIMU=* FIFOOverrunIMU=* BusErrorIMU=* BusTimeoutIMU=*
	 */
	while(1){
		read_mem(page << 8, buf, 512);
		if(0 != buf[511]){
			break;
		}
		/* Error Statuses */
		printf("#BufferOverrunIMU=%d FIFOOverrunIMU=%d BusErrorIMU=%d BusTimeoutIMU=%d\r\n",
		       buf[sizeof(imu_data_t) + 0], buf[sizeof(imu_data_t) + 1],
		       buf[sizeof(imu_data_t) + 2], buf[sizeof(imu_data_t) + 3]);
		delay_ms(1);
		/* Magnetometer Data */
		printf("M, %f, %d, %d, %d, %d, %d, %d\r\n",
		       page * 0.01, data->mag.x, data->mag.y, data->mag.z,
		       data->temperature, data->tip_pressed, page);
		delay_ms(1);
		/* IMU Accelerometer Data */
		for(i=0; i < data->num_accel; i++, ia_sample++){
			printf("IA, %f, %d, %d, %d, %d, %d, %d\r\n",
			       (float) ia_sample / 1600, data->accel[i].x, data->accel[i].y, data->accel[i].z,
			       data->temperature, data->tip_pressed, page);
			delay_ms(1);
		}
		/* Gyroscope Data */
		for(i=0; i < data->num_gyro; i++, g_sample++){
			printf("G, %f, %d, %d, %d, %d, %d, %d\r\n",
			       (float) g_sample / 760, data->gyro[i].x, data->gyro[i].y, data->gyro[i].z,
			       data->temperature, data->tip_pressed, page);
			delay_ms(1);
		}
		/* Auxiliary accelerometer data. */
		for(i=0; i < data->num_aux; i++, aa_sample++){
			printf("AA, %f, %d, %d, %d, %d, %d, %d\r\n",
			       (float) aa_sample / 1600, data->aux[i].x, data->aux[i].y, data->aux[i].z,
			       data->temperature, data->tip_pressed, page);
			delay_ms(1);
		}
		page += 2;
	}
	printf("#Done at page %d.\r\n", page);
	fflush(stdout);
}

int main(void){
	uint8_t status;
	status = init_system();
	print_status_message(status);

	led_on();
	
	/* Bluetooth test code. */
	/* TODO: REQn and RDYn macros. */
	gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO0); /* RDYn */
	GPIOA_BSRR = GPIO7;
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO7); /* REQNn */
	//setup_spi(0, 0, SPI_CR1_BAUDRATE_FPCLK_DIV_16, SPI_CR1_LSBFIRST);
	setup_spi(0, 0, SPI_CR1_BAUDRATE_FPCLK_DIV_256, SPI_CR1_LSBFIRST);
	//TODO
	write_str("Waiting for RDYn to be low... ");
	GPIOA_BSRR = (GPIO7 << 16);
	while(GPIOB_IDR &= GPIO0){
		/* Wait to for RDYn to be low. */
	}
	write_str("RDYn is low.\r\n");
	uint8_t buf[34];
	rx_spi(buf, 34);
	GPIOA_BSRR = GPIO7;
	printf("Data received: ");
	for(int i=0;i<34;i++){
		printf("0x%02X ", buf[i]);
	}
	printf("\r\n");
	fflush(stdout);
	while(1);
	
	

	/* IMU driver test code.
	 * The data rate is too fast for the USART, so the data is recorded to
	 * the external flash memory and read out later. */
	enum {ERASE, RECORD, PLAYBACK, TEST, COMBO} mode = COMBO;

	if(COMBO == mode){
		/* Record data for 1s and read it out. */
		imu_erase();
		imu_record(100); /* NOTE: if record time is increased beyond 1s, increase
		                  * the number of sectors erased by imu_erase(). */
		imu_playback();
		led_off();
	} else if(TEST == mode){
		imu_test();
	} else if(ERASE == mode){
		imu_erase();
	} else if(RECORD == mode){
		imu_record(0);
	} else if(mode == PLAYBACK){
		imu_playback();
	}
	while(1);
		
}

/* Print a welcome message using write_str(). */
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
	if(init_aux_accel())status |= ERROR_AUXACCEL;
	if(1)status |= ERROR_BLUETOOTH;
	if(init_memory())status |= ERROR_MEMORY;
	if(init_imu())status |= ERROR_IMU;
	//TODO
	//if(init_battery())status |= ERROR_BATTERY;
	return status;
}

/* Shutdown all board peripherals.
 * TODO: uC shutdown and button wakeup. */
void shutdown_system(void){
	shutdown_battery();
	shutdown_memory();
	//TODO: uC shutdown and button wakeup.
}