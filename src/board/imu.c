/* LSM9DS0TR 9-axis IMU driver.
 * This file provides a high-level interface to the IMU, including a timer-based
 * (TIM1) task that automatically streams data from the sensor.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 */
#include "imu.h"
#include "../peripherals/i2c.h"
#include "../peripherals/systick.h"
#include "../peripherals/housekeeping.h"
#include "../peripherals/clock.h"
#include "switches.h"
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <math.h>

//TODO
#include <stdio.h>
#include "led.h"

/* I2C address of LSM9DS0TR accelerometer with SDO_XM high. */
#define IMU_ACCEL_ADDR 0x1D
/* I2C address of LSM9DS0TR gyroscope with SDO_G high. */
#define IMU_GYRO_ADDR 0x6B

/* Current Temperature of IMU. This is the raw value from the IMU.
 * If no data is available, it will be set to 0xFFFF. */
int16_t IMUTemperature;

/* Current Temperature of IMU, in degrees Celsius. If data is unavailable,
 * it will be set to NaN. */
static void update_imu_temp(void){
	static volatile uint8_t done_h, done_l, tempdata[2];
	i2c_ticket_t ticket;
	
	if(I2C_DONE == done_l && I2C_DONE == done_h){
		IMUTemperature = (int16_t)tempdata[0];
	} else {
		IMUTemperature = 0xFFFF;
	}

	/* Temperature high byte must be read first. */
	ticket.rw = I2C_READ;
	ticket.addr = IMU_ACCEL_ADDR; /* LSM9DS0TR accelerometer I2C address with SDO_XM high. */
 	ticket.reg = 0x06; /* OUT_TEMP_H_XM. */
	ticket.data = &tempdata[1];
	ticket.size = 1;
	ticket.done_flag = &done_h;
	ticket.at_time = 0;
	ticket.done_callback = 0;
	done_h = 0;
	add_ticket_i2c(&ticket);

	ticket.reg = 0x05; /* OUT_TEMP_L_XM. */
	ticket.data = &tempdata[0];
	ticket.done_flag = &done_l;
	done_l = 0;
	add_ticket_i2c(&ticket);
}

/* If true, data loss occurred because a data fetch started while the last
 * buffer was in use. */
bool BufferOverrunIMU;

/* If true, data loss occurred due to an overrun in an IMU internal FIFO. */
bool FIFOOverrunIMU;

/* If true, data loss occurred due to a bus error.
 * TODO: Develop mitigation strategies (such as retrying the transfer). */
bool BusErrorIMU;

/* If true, data loss occurred due to the data fetch not completing in time. */
bool BusTimeoutIMU;

/* Data streaming task sequence of operations:
 *  -Triggered by interrupt every 10ms:
 *   -Fetch number of data points in accelerometer FIFO
 *   -Fetch data from accelerometer
 *   -Fetch number of data points in gyroscope FIFO
 *   -Fetch data from gyro
 *   -Fetch magnetometer reading
 */

/* IMU Data Buffers */
static volatile imu_data_t Buffer[2];
static volatile enum {BUF_STALE, BUF_FRESH, BUF_FRESHEST, BUF_READING, BUF_WRITING}
                      BufferState[2] = {BUF_STALE, BUF_STALE}; /* State of data buffers. */
static volatile imu_data_t *CurrentBuf; /* The buffer currently being filled with data. */
static volatile uint8_t CurrentBufIndex; /* The index of the buffer currently
                                          * being filled with data. */

/* These variables are used by all the streaming task functions. */
static volatile i2c_ticket_t Ticket;
static volatile bool TaskComplete; /* The streaming task finished successfully. */
static volatile uint8_t Done; /* Done value of latest i2c transaction. Used to
                               * detect errors. */

static void fetch_imu_accel_num(void);
static void fetch_imu_accel_data(void);
static void fetch_imu_gyro_num(void);
static void fetch_imu_gyro_data(void);
static void fetch_imu_mag(void);
static void fetch_finish(void);

/* Get the number of data point in the IMU accelerometer's FIFO. */
static void fetch_imu_accel_num(void){
	Ticket.addr = IMU_ACCEL_ADDR;
	Ticket.reg = 0x2F; /* FIFO_SRC_REG */
	Ticket.data = &CurrentBuf->num_accel; /* NOTE: The number of points in the
	                                       * FIFO will need to be extracted from
					       * the register data. */
	Ticket.size = 1;
	Ticket.done_callback = &fetch_imu_accel_data;
	add_ticket_i2c((i2c_ticket_t *)&Ticket);
}

/* Get data points from the IMU accelerometer's FIFO. */
static void fetch_imu_accel_data(void){
	/* Extract number of points in FIFO from FIFO_SRC_REG. */
	if(CurrentBuf->num_accel & 0x40){ /* OVRN bit. */
		FIFOOverrunIMU = true;
	}
	CurrentBuf->num_accel &= 0x1F;

	Ticket.addr = IMU_ACCEL_ADDR;
	Ticket.reg = 0xA8; /* OUT_X_L_A with auto-increment bit set */
	Ticket.data = &CurrentBuf->accel;
	Ticket.size = CurrentBuf->num_accel * sizeof(vec3_t);
	Ticket.done_callback = &fetch_imu_gyro_num;
	add_ticket_i2c((i2c_ticket_t *)&Ticket);
}

/* Get the number of data points in the IMU gyroscope's FIFO. */
static void fetch_imu_gyro_num(void){
	Ticket.addr = IMU_GYRO_ADDR;
	Ticket.reg = 0x2F; /* FIFO_SRC_REG_G */
	Ticket.data = &CurrentBuf->num_gyro; /* NOTE: The number of points in the
	                                      * FIFO will need to be extracted from
	                                      * the register data. */
	Ticket.size = 1;
	Ticket.done_callback = &fetch_imu_gyro_data;
	add_ticket_i2c((i2c_ticket_t *)&Ticket);
}

/* Get data points from the IMU gyroscope's FIFO. */
static void fetch_imu_gyro_data(void){
	/* Extract number of points in FIFO from FIFO_SRC_REG. */
	if(CurrentBuf->num_gyro & 0x40){ /* OVRN bit. */
		FIFOOverrunIMU = true;
	}
	CurrentBuf->num_gyro &= 0x1F;
	
	Ticket.addr = IMU_GYRO_ADDR;
	Ticket.reg = 0xA8; /* OUT_X_L_G with auto-increment bit set */
	Ticket.data = &CurrentBuf->gyro;
	Ticket.size = CurrentBuf->num_gyro * sizeof(vec3_t);
	Ticket.done_callback = &fetch_imu_mag;
	add_ticket_i2c((i2c_ticket_t *)&Ticket);
}

/* Get magnetometer data. */
static void fetch_imu_mag(void){
	Ticket.addr = IMU_ACCEL_ADDR;
	Ticket.reg = 0x88; /* OUT_X_L_M with auto-increment bit set */
	Ticket.data = &CurrentBuf->mag;
	Ticket.size = sizeof(vec3_t);
	Ticket.done_callback = &fetch_finish;
	Ticket.at_time = &CurrentBuf->mag_time;
	add_ticket_i2c((i2c_ticket_t *)&Ticket);
}

/* Finish the current data fetch. */
static void fetch_finish(void){
	/* Store tip switch state. */
	CurrentBuf->tip_pressed = tip_switch_pressed();

	/* Store current IMU temperature. */
	CurrentBuf->temperature = IMUTemperature;

	if(BUF_FRESH == BufferState[0] || BUF_FRESH == BufferState[1]){
		BufferState[CurrentBufIndex] = BUF_FRESHEST;
	} else {
		BufferState[CurrentBufIndex] = BUF_FRESH;
	}
	TaskComplete = true;
}

/* Data streaming task interrupt. Called every 10ms by TIM1. */
void tim1_up_tim16_isr(void){
	TIM1_SR = 0; /* Clear the interrupt flag so the ISR will exit. */
	
	if(false == TaskComplete){
		if(I2C_DONE == Done){
			/* Previous fetch has not completed, but a bus
			 * error has not occurred. */
			BusTimeoutIMU = true;
			return;
		} else {
			/* Previous fetch has not completed due to a bus error. */
			BusErrorIMU = true;
		}
	}
	
	if(BUF_STALE == BufferState[0]){
		BufferState[0] = BUF_WRITING;
		CurrentBuf = &Buffer[0];
		CurrentBufIndex = 0;
	} else if(BUF_STALE == BufferState[1]){
		BufferState[1] = BUF_WRITING;
		CurrentBuf = &Buffer[1];
		CurrentBufIndex = 1;
	} else {
		/* No buffer available. */
		BufferOverrunIMU = true;
		TaskComplete = true;
		return;
	}

	Ticket.rw = I2C_READ;
	Ticket.at_time = NULL;
	Ticket.done_flag = &Done;

	TaskComplete = false;
	Done = I2C_BUSY;
	fetch_imu_accel_num();
}

/* Setup TIM1 and start the data streaming task. */
static void start_stream_task(void){
	TaskComplete = true; /* Prevent confusion during the first run. */
	rcc_periph_clock_enable(RCC_TIM1);
	TIM1_CR1 = TIM_CR1_URS; /* Only overflow generates an interrupt. */
	TIM1_DIER = TIM_DIER_UIE; /* Enable update event interrupt generation. */
	TIM1_PSC = SystemClock / 10000; /* Set prescaler to generate a 10kHz clock. */
	TIM1_ARR = 99; /* (Timer counts from 0.) Set timer to generate an update
	                * event at 100Hz. */
	TIM1_CR1 |= TIM_CR1_CEN; /* Start timer. */
	nvic_set_priority(NVIC_TIM1_UP_TIM16_IRQ, IMU_IRQ_PRIORITY << 4);
	nvic_enable_irq(NVIC_TIM1_UP_TIM16_IRQ);
}

/* Returns a pointer to an IMU data structure if fresh data is available.
 * Otherwise, NULL is returned. Copy data out of the buffer as quickly as
 * possible and call release_buf_imu() when done! */
imu_data_t *get_buf_imu(void){
	if(BUF_FRESHEST == BufferState[0]){
		BufferState[0] = BUF_READING;
		return (imu_data_t *) &Buffer[0];
	} else
	if(BUF_FRESHEST == BufferState[1]){
		BufferState[1] = BUF_READING;
		return (imu_data_t *) &Buffer[1];
	} else
	if(BUF_FRESH == BufferState[0]){
		BufferState[0] = BUF_READING;
		return (imu_data_t *) &Buffer[0];
	} else
	if(BUF_FRESH == BufferState[1]){
		BufferState[1] = BUF_READING;
		return (imu_data_t *) &Buffer[1];
	} else {
		return NULL;
	}
}

/* Tell the IMU stream task that data has been copied out of the buffer
 * provided by get_buf_imu(). */
void release_buf_imu(void){
	if(BUF_READING == BufferState[0]){
		BufferState[0] = BUF_STALE;
	}
	if(BUF_READING == BufferState[1]){
		BufferState[1] = BUF_STALE;
	}
}

/* Initialize the IMU and start the data streaming task.
 * Returns 0 on success, -1 on error.
 * This should be called before initializing the battery gas gauge to reduce
 * traffic on the I2C bus during accelerometer-gyroscope synchronization. */
int init_imu(void){
	i2c_ticket_t a_ticket, g_ticket;
	volatile uint8_t a_done, g_done;
	volatile uint8_t a_data, g_data;
	
	a_ticket.addr = IMU_ACCEL_ADDR; /* LSM9DS0TR accelerometer I2C address with SDO_XM high. */
	a_ticket.done_flag = &a_done;
	a_ticket.at_time = 0;
	a_ticket.done_callback = 0;
	a_ticket.data = &a_data;
	a_ticket.size = 1;

	g_ticket.addr = IMU_GYRO_ADDR; /* LSM9DS0TR gyroscope I2C address with SDO_G high. */
	g_ticket.done_flag = &g_done;
	g_ticket.at_time = 0;
	g_ticket.done_callback = 0;
	g_ticket.data = &g_data;
	g_ticket.size = 1;

	/* Test that the accelerometer and gyroscope are responding. */
	a_ticket.rw = I2C_READ;
	a_ticket.reg = 0x0F; /* WHO_AM_I_XM */
	g_ticket.rw = I2C_READ;
	g_ticket.reg = 0x0F; /* WHO_AM_I_G */
	a_done = 0;
	g_done = 0;
	add_ticket_i2c(&a_ticket);
	add_ticket_i2c(&g_ticket);
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
	a_ticket.rw = I2C_WRITE;
	a_ticket.reg = 0x1F; /* CTRL_REG0_XM */
	a_data = 0x80; /* Set reboot flag. */
	g_ticket.rw = I2C_WRITE;
	g_ticket.reg = 0x24; /* CTRL_REG5_G */
	g_data = 0x80; /* Set reboot flag. */
	a_done = 0;
	g_done = 0;
	add_ticket_i2c(&a_ticket);
	add_ticket_i2c(&g_ticket);
	while(I2C_BUSY == a_done || I2C_BUSY == g_done){
		/* Wait for tickets to be processed. */
	}
	if(I2C_DONE != a_done || I2C_DONE != g_done){
		/* Bus error. */
		return -1;
	}
	delay_ms(10); /* Give the device time to reset itself. */

	/* Enable FIFO */
	a_ticket.rw = I2C_WRITE;
	a_ticket.reg = 0x1F; /* CTRL_REG0_XM */
	a_data = 0x40; /* Enable FIFO */
	g_ticket.rw = I2C_WRITE;
	g_ticket.reg = 0x24; /* CTRL_REG5_G */
	/* TODO: Gyroscope: decide on range, output data selection (filters). */
	g_data = 0x40; /* Enable FIFO */
	a_done = 0;
	g_done = 0;
	add_ticket_i2c(&a_ticket);
	add_ticket_i2c(&g_ticket);
	while(I2C_BUSY == a_done || I2C_BUSY == g_done){
		/* Wait for tickets to be processed. */
	}
	if(I2C_DONE != a_done || I2C_DONE != g_done){
		/* Bus error. */
		return -1;
	}

	/* Setup FIFO modes. */
	a_ticket.rw = I2C_WRITE;
	a_ticket.reg = 0x2E; /* FIFO_CTRL_REG */
	a_data = 0x5F; /* FIFO stream mode and watermark level=31 */
	g_ticket.rw = I2C_WRITE;
	g_ticket.reg = 0x2E; /* FIFO_CTRL_REG_G */
	g_data = 0x5F; /* FIFO stream mode and watermark level=31 */
	a_done = 0;
	g_done = 0;
	add_ticket_i2c(&a_ticket);
	add_ticket_i2c(&g_ticket);
	while(I2C_BUSY == a_done || I2C_BUSY == g_done){
		/* Wait for tickets to be processed. */
	}
	if(I2C_DONE != a_done || I2C_DONE != g_done){
		/* Bus error. */
		return -1;
	}

	/* Enable Magnetometer and Temperature sensor. */
	a_ticket.rw = I2C_WRITE;
	a_ticket.reg = 0x24; /* CTRL_REG5_XM */
	a_data = 0xF4; /* Enable temp. sensor, magnetometer high resolution mode,
	                * magnetometer ODR = 100Hz. */
	a_done = 0;
	add_ticket_i2c(&a_ticket);
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
	a_ticket.rw = I2C_WRITE;
	a_ticket.reg = 0x20; /* CTRL_REG1_XM */
	a_data = 0xA7; /* 1600Hz ODR and enable all axises. */
	g_ticket.rw = I2C_WRITE;
	g_ticket.reg = 0x20; /* CTRL_REG1_G */
	g_data = 0xFF; /* 760Hz ODR, 100Hz BW, power up, and enable all axises. */
	a_done = 0;
	g_done = 0;
	add_ticket_i2c(&a_ticket);
	add_ticket_i2c(&g_ticket);
	while(I2C_BUSY == a_done || I2C_BUSY == g_done){
		/* Wait for tickets to be processed. */
	}
	if(I2C_DONE != a_done || I2C_DONE != g_done){
		/* Bus error. */
		return -1;
	}

	start_stream_task();
	
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