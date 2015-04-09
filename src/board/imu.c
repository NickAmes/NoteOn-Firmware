/* LSM9DS0TR 9-axis IMU and LIS3DSHTR 3-axis auxiliary accelerometer driver.
 * This file provides a high-level interface to the IMU and auxiliary
 * accelerometer, including a timer-based (TIM1) task that automatically
 * streams data from the sensor.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * NoteOn Contains code from the libopencm3 and newlib projects.              */
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

/* I2C address of LSM9DS0TR accelerometer with SDO_XM high. */
#define IMU_ACCEL_ADDR 0x1D
/* I2C address of LSM9DS0TR gyroscope with SDO_G high. */
#define IMU_GYRO_ADDR 0x6B
/* I2C address of LIS3DSHTR aux. accelerometer with SEL low. */
#define AUX_ACCEL_ADDR 0x1E

/* Current Temperature of IMU. This is the raw value from the IMU.
 * If no data is available, it will be set to 0xFFFF. */
int16_t IMUTemperature = 0xFFFF;

/* Retrieve current temperature of IMU. */
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
static void fetch_aux_accel_num(void);
static void fetch_aux_accel_data(void);
static void fetch_finish(void);

/* Get the number of data points in the IMU accelerometer's FIFO. */
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
	if(CurrentBuf->num_accel > MAX_IMU_ACCEL_POINTS){
		CurrentBuf->num_accel = MAX_IMU_ACCEL_POINTS;
	}

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
	if(CurrentBuf->num_gyro > MAX_IMU_GYRO_POINTS){
		CurrentBuf->num_gyro = MAX_IMU_GYRO_POINTS;
	}
	
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
	Ticket.done_callback = &fetch_aux_accel_num;
	Ticket.at_time = &CurrentBuf->mag_time;
	add_ticket_i2c((i2c_ticket_t *)&Ticket);
}

/* Get the number of data points in the aux. accelerometer's FIFO. */
static void fetch_aux_accel_num(void){
	Ticket.addr = AUX_ACCEL_ADDR;
	Ticket.reg = 0x2F; /* FIFO_SRC */
	Ticket.data = &CurrentBuf->num_aux; /* NOTE: The number of points in the
	                                     * FIFO will need to be extracted from
					     * the register data. */
	Ticket.size = 1;
	Ticket.done_callback = &fetch_aux_accel_data;
	add_ticket_i2c((i2c_ticket_t *)&Ticket);
}

/* Get data points from the aux. accelerometer's FIFO. */
static void fetch_aux_accel_data(void){
	/* Extract number of points in FIFO from FIFO_SRC. */
	if(CurrentBuf->num_aux & 0x40){ /* OVRN bit. */
		FIFOOverrunIMU = true;
	}
	CurrentBuf->num_aux &= 0x1F;
	if(CurrentBuf->num_aux > MAX_IMU_ACCEL_POINTS){
		CurrentBuf->num_aux = MAX_IMU_ACCEL_POINTS;
	}

	Ticket.addr = AUX_ACCEL_ADDR;
	Ticket.reg = 0x28; /* OUT_X_L */
	Ticket.data = &CurrentBuf->aux;
	Ticket.size = CurrentBuf->num_aux * sizeof(vec3_t);
	Ticket.done_callback = &fetch_finish;
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
		/* Previous fetch not complete. */
		if(I2C_DONE == Done){
			/* Previous fetch has not completed, but a bus
			 * error has not occurred. It's probably still in
			 * progress. */
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
 * Otherwise, NULL is returned.
 * NOTE: Copy data out of the buffer as quickly as
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

/* Initialize the LSM9DS0TR IMU. start_imu() must be called after this function
 * to synchronize the IMU and aux. accelerometer and start the data streaming task.
 * Returns 0 on success, -1 on error. */
int init_imu(void){
	/* Test that the accelerometer and gyroscope are responding. */
	uint8_t a_data, g_data;
	
	if(I2C_DONE != read_byte_i2c(IMU_ACCEL_ADDR, 0x0F, /* WHO_AM_I_XM */
	               &a_data)){
		return -1; /* Bus error. */
	}
	if(I2C_DONE != read_byte_i2c(IMU_GYRO_ADDR, 0x0F, /* WHO_AM_I_G */
	               &g_data)){
		return -1; /* Bus error. */
	}

	if(0x49 != a_data || 0xD4 != g_data){
		/* Incorrect WHO_AM_I values. */
		return -1;
	}
	

	/* Soft reset. */
	if(I2C_DONE != write_byte_i2c(IMU_ACCEL_ADDR, 0x1F, /* CTRL_REG0_XM */
	               0x80)){ /* Set reboot flag. */
		return -1; /* Bus error. */
	}
	if(I2C_DONE != write_byte_i2c(IMU_GYRO_ADDR, 0x24, /* CTRL_REG5_G */
	               0x80)){ /* Set reboot flag. */
		return -1; /* Bus error. */
	}
	delay_ms(10); /* Give the device time to reset itself. */
	
	/* Enable FIFO */
	if(I2C_DONE != write_byte_i2c(IMU_ACCEL_ADDR, 0x1F, /* CTRL_REG0_XM */
	               0x40)){ /* Enable FIFO */
		return -1; /* Bus error. */
	}
	if(I2C_DONE != write_byte_i2c(IMU_GYRO_ADDR, 0x24, /* CTRL_REG5_G */
	               0x40)){ /* Enable FIFO */
		return -1; /* Bus error. */
	}

	/* Setup FIFO modes. */
	if(I2C_DONE != write_byte_i2c(IMU_ACCEL_ADDR, 0x2E, /* FIFO_CTRL_REG */
	               0x5F)){ /* FIFO stream mode and watermark level=31 */
		return -1; /* Bus error. */
	}
	if(I2C_DONE != write_byte_i2c(IMU_GYRO_ADDR, 0x2E, /* FIFO_CTRL_REG_G */
	               0x5F)){ /* FIFO stream mode and watermark level=31 */
		return -1; /* Bus error. */
	}

	/* Enable Magnetometer and Temperature sensor. */
	if(I2C_DONE != write_byte_i2c(IMU_ACCEL_ADDR, 0x24,   /* CTRL_REG5_XM */
                       0xF4)){ /* Enable temp. sensor, magnetometer high resolution mode,
		                * magnetometer ODR = 100Hz. */
		return -1; /* Bus error. */
	}

	/* TODO: Try enabling BDU mode for the benefit of the magnetometer. */
	/* TODO: Filter settings. */
	return 0;
}

/* Initialize the LIS3DSHTR auxiliary accelerometer. start_imu() must be called
 * after this function to synchronize the IMU and aux. accelerometer and start
 * the data streaming task.
 * Returns 0 on success, -1 on error. */
int init_aux_accel(void){
	/* Check that the axillary accelerometer is responding. */
	uint8_t a_data;
	if(I2C_DONE != read_byte_i2c(AUX_ACCEL_ADDR, 0x0F, /* WHO_AM_I */
	               &a_data)){
		return -1; /* Bus error. */
	}
	if(0x3F != a_data){
		/* Incorrect WHO_AM_I values. */
		return -1;
	}

	/* Soft reset. */
	if(I2C_DONE != write_byte_i2c(AUX_ACCEL_ADDR, 0x25, /* CTRL_REG6 */
	               0x80)){ /* Set reboot flag. */
		return -1; /* Bus error. */
	}
	delay_ms(10); /* Give the device time to reset itself. */

	/* Enable FIFO and address auto-increment. */
	if(I2C_DONE != write_byte_i2c(AUX_ACCEL_ADDR, 0x25, /* CTRL_REG6 */
	               0x50)){ /* Set FIFO and address auto-increment bits. */
		return -1; /* Bus error. */
	}
	
	/* Setup FIFO modes. */
	if(I2C_DONE != write_byte_i2c(AUX_ACCEL_ADDR, 0x2E, /* FIFO_CTRL */
	               0x5F)){ /* FIFO stream mode and watermark level=31 */
		return -1; /* Bus error. */
	}

	return 0;
}
	
/* Synchronize the IMU and aux. accelerometer and start the data streaming
 * and temperature update tasks. */
void start_imu(void){
	/* Set ODR and bring IMU and aux. accelerometer out of sleep mode.
	 * This is the critical step that synchronizes the outputs of the
	 * accelerometers and gyroscope. */
	volatile uint8_t a_done = 0, g_done = 0, aux_done = 0;
	uint8_t a_data, g_data, aux_data;
	a_data = 0xA7; /* 1600Hz ODR and enable all axises. */
	g_data = 0xFF; /* 760Hz ODR, 100Hz BW, power up, and enable all axises. */
	aux_data = 0x97; /* 1600Hz ODR and enable all axises. */
	add_ticket_i2c_f(I2C_WRITE, IMU_ACCEL_ADDR, 0x20, /* CTRL_REG1_XM */
			 &a_data, 1, &a_done, NULL, NULL);
	add_ticket_i2c_f(I2C_WRITE, AUX_ACCEL_ADDR, 0x20, /* CTRL_REG4 */
			 &aux_data, 1, &aux_done, NULL, NULL);
	add_ticket_i2c_f(I2C_WRITE, IMU_GYRO_ADDR, 0x20, /* CTRL_REG1_G */
			 &g_data, 1, &g_done, NULL, NULL);

	while(I2C_BUSY == a_done || I2C_BUSY == g_done || I2C_BUSY == aux_done){
		/* Wait for tickets to be processed. */
	}
	if(I2C_DONE != a_done || I2C_DONE != g_done || I2C_DONE != aux_done){
		/* Bus error. */
		BusErrorIMU = true;
		return;
	}

	start_stream_task();

	/* Start temperature update task. */
	HousekeepingTasks[IMU_TEMP_HK_SLOT] = &update_imu_temp;
}

/* Shutdown the IMU data streaming task and put the IMU and aux. accelerometer
 * in a low-power state. */
void shutdown_imu(void){
	/* Disable temperature sensing housekeeping task. */
	HousekeepingTasks[IMU_TEMP_HK_SLOT] = 0;

	/* Stop data stream task. */
	nvic_disable_irq(NVIC_TIM1_UP_TIM16_IRQ);

	/* Shutdown power to the data stream task timer. */
	rcc_periph_clock_disable(RCC_TIM1);

	/* Put the IMU into power-down mode. */
	write_byte_i2c(IMU_ACCEL_ADDR, 0x20, /* CTRL_REG1_XM */
		       0x00); /* Power-down mode. */
	write_byte_i2c(IMU_GYRO_ADDR,  0x20, /* CTRL_REG1_G */
		       0x00); /* Power-down mode. */

	/* Put the auxiliary accelerometer into power-down mode. */
	write_byte_i2c(AUX_ACCEL_ADDR, 0x20, /* CTRL_REG4 */
		       0x00); /* Power-down mode. */
}
