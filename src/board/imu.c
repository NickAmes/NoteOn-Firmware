/* LSM9DS0TR 9-axis IMU driver.
 * This file provides a high-level interface to the IMU, including a timer-based
 * task that automatically streams data from the sensor.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 */
#include "imu.h"
#include "../peripherals/i2c.h"
#include "../peripherals/systick.h"
#include "../peripherals/housekeeping.h"
#include <math.h>

//TODO
#include <stdio.h>

/* Current Temperature of IMU, in degrees Celsius. */
float IMUTemperature;

/* Current Temperature of IMU, in degrees Celsius. If data is unavailable,
 * it will be set to NaN. */
static void update_imu_temp(void){
	static volatile uint8_t done, tempdata[2];
	i2c_ticket_t ticket;
	
	if(I2C_DONE == done){
		/* TODO: Convert temperature data. */
		IMUTemperature = tempdata[0];
		iprintf("tempdata[1] = 0x%X tempdata[0] = 0x%X\r\n", tempdata[1], tempdata[0]);
	} else {
		IMUTemperature = NAN;
	}

	ticket.rw = I2C_READ;
	ticket.addr = 0x1D; /* LSM9DS0TR accelerometer I2C address with SDO_XM high. */
	ticket.reg = 0x05; /* OUT_TEMP_L_XM with auto-increment set. */
	ticket.data = &tempdata[0];
	ticket.size = 2;
	ticket.done_flag = &done;
	ticket.at_time = 0;
	
	done = 0;
	add_ticket_i2c(&ticket);
}

/* If true, data loss occurred because a data fetch started while the last
 * buffer was in use. This should be set back to false after the main task
 * recognizes the error. */
bool OverrunIMU;

/* If true, data loss occurred due to a bus error.
 * TODO: Develop mitigation strategies (such as retrying the transfer). */
bool BusErrorIMU;

/* Returns a pointer to an IMU data structure if fresh data is available.
 * Otherwise, NULL is returned. Copy data out of the buffer as quickly as
 * possible and call release_buf_imu() when done! */
imu_data_t *get_buf_imu(void){
	return 0;
}

/* Tell the IMU stream task that data has been copied out of the buffer
 * provided by get_buf_imu(). */
void release_buf_imu(void){

}

/* Initialize the IMU and start the data streaming task.
 * Returns 0 on success, -1 on error.
 * This should be called before initializing the battery gas gauge to reduce
 * traffic on the I2C bus during accelerometer-gyroscope synchronization. */
int init_imu(void){
	i2c_ticket_t accel_t, gyro_t;
	volatile uint8_t a_done, g_done;
	volatile uint8_t a_data, g_data;
	
	accel_t.addr = 0x1D; /* LSM9DS0TR accelerometer I2C address with SDO_XM high. */
	accel_t.done_flag = &a_done;
	accel_t.at_time = 0;
	accel_t.data = &a_data;
	accel_t.size = 1;

	gyro_t.addr = 0x6B; /* LSM9DS0TR gyroscope I2C address with SDO_G high. */
	gyro_t.done_flag = &g_done;
	gyro_t.at_time = 0;
	gyro_t.data = &g_data;
	gyro_t.size = 1;

	/* Test that the accelerometer and gyroscope are responding. */
	accel_t.rw = I2C_READ;
	accel_t.reg = 0x0F; /* WHO_AM_I_XM */
	gyro_t.rw = I2C_READ;
	gyro_t.reg = 0x0F; /* WHO_AM_I_G */
	a_done = 0;
	g_done = 0;
	add_ticket_i2c(&accel_t);
	add_ticket_i2c(&gyro_t);
	while(I2C_BUSY == a_done || I2C_BUSY == g_done){
		/* Wait for tickets to be processed. */
	}
	if(I2C_DONE != a_done || I2C_DONE != g_done){
		/* Bus error. */
		return -1;
	}
	if(0x49 != a_data || 0xD4 != g_data){
		/* Incorrect WHO_AM_I values. */
		return -1;
	}

	/* Soft reset. */
	accel_t.rw = I2C_WRITE;
	accel_t.reg = 0x1F; /* CTRL_REG0_XM */
	a_data = 0x80; /* Set reboot flag. */
	gyro_t.rw = I2C_WRITE;
	gyro_t.reg = 0x24; /* CTRL_REG5_G */
	g_data = 0x80; /* Set reboot flag. */
	a_done = 0;
	g_done = 0;
	add_ticket_i2c(&accel_t);
	add_ticket_i2c(&gyro_t);
	while(I2C_BUSY == a_done || I2C_BUSY == g_done){
		/* Wait for tickets to be processed. */
	}
	if(I2C_DONE != a_done || I2C_DONE != g_done){
		/* Bus error. */
		return -1;
	}

	/* Enable FIFO */
	accel_t.rw = I2C_WRITE;
	accel_t.reg = 0x1F; /* CTRL_REG0_XM */
	a_data = 0x40; /* Enable FIFO */
	gyro_t.rw = I2C_WRITE;
	gyro_t.reg = 0x24; /* CTRL_REG5_G */
	/* TODO: Gyroscope: decide on range, output data selection (filters). */
	g_data = 0x40; /* Enable FIFO */
	a_done = 0;
	g_done = 0;
	add_ticket_i2c(&accel_t);
	add_ticket_i2c(&gyro_t);
	while(I2C_BUSY == a_done || I2C_BUSY == g_done){
		/* Wait for tickets to be processed. */
	}
	if(I2C_DONE != a_done || I2C_DONE != g_done){
		/* Bus error. */
		return -1;
	}

	/* Setup FIFO modes. */
	accel_t.rw = I2C_WRITE;
	accel_t.reg = 0x2E; /* FIFO_CTRL_REG */
	a_data = 0x5F; /* FIFO stream mode and watermark level=31 */
	gyro_t.rw = I2C_WRITE;
	gyro_t.reg = 0x2E; /* FIFO_CTRL_REG_G */
	g_data = 0x5F; /* FIFO stream mode and watermark level=31 */
	a_done = 0;
	g_done = 0;
	add_ticket_i2c(&accel_t);
	add_ticket_i2c(&gyro_t);
	while(I2C_BUSY == a_done || I2C_BUSY == g_done){
		/* Wait for tickets to be processed. */
	}
	if(I2C_DONE != a_done || I2C_DONE != g_done){
		/* Bus error. */
		return -1;
	}

	/* Enable Magnetometer and Temperature sensor. */
	accel_t.rw = I2C_WRITE;
	accel_t.reg = 0x20; /* CTRL_REG5_XM */
	a_data = 0xF4; /* Enable temp. sensor, magnetometer high resolution mode,
	                * magnetometer ODR = 100Hz. */
	a_done = 0;
	add_ticket_i2c(&accel_t);
	while(I2C_BUSY == a_done){
		/* Wait for ticket to be processed. */
	}
	if(I2C_DONE != a_done){
		/* Bus error. */
		return -1;
	}

	/* Set ODR and bring IMU out of sleep mode.
	 * This is the critical step that synchronizes the outputs of the
	 * accelerometer and gyroscope. */
	accel_t.rw = I2C_WRITE;
	accel_t.reg = 0x20; /* CTRL_REG1_XM */
	a_data = 0xA7; /* 1600Hz ODR and enable all axises. */
	gyro_t.rw = I2C_WRITE;
	gyro_t.reg = 0x20; /* CTRL_REG1_G */
	g_data = 0xFF; /* 760Hz ODR, 100Hz BW, power up, and enable all axises. */
	a_done = 0;
	g_done = 0;
	add_ticket_i2c(&accel_t);
	add_ticket_i2c(&gyro_t);
	while(I2C_BUSY == a_done || I2C_BUSY == g_done){
		/* Wait for tickets to be processed. */
	}
	if(I2C_DONE != a_done || I2C_DONE != g_done){
		/* Bus error. */
		return -1;
	}

	/* TODO: Start IMU data streaming task. */
	
	/* Start temperature update task. */
	HousekeepingTasks[IMU_TEMP_HK_SLOT] = &update_imu_temp;

	/* TODO: Try enabling BDU mode for the benefit of the magnetometer. */

	return 0;
}
	

/* Shutdown the IMU data streaming task and put the IMU in a low-power state. */
void shutdown_imu(void){
	/* Disable temperature sensing housekeeping task. */
	HousekeepingTasks[IMU_TEMP_HK_SLOT] = 0;

	/* TODO: Stop stream task and wait for it to stop. */

	/* TODO: Put the IMU in a low-power state. */
}