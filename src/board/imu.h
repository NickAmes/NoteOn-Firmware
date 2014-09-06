/* LSM9DS0TR 9-axis IMU and LIS3DSHTR 3-axis auxiliary accelerometer driver.
 * This file provides a high-level interface to the IMU and auxiliary
 * accelerometer, including a timer-based (TIM1) task that automatically
 * streams data from the sensor.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * NoteOn Contains code from the libopencm3 and newlib projects.              */
#ifndef IMU_H
#define IMU_H
#include <stdint.h>
#include <stdbool.h>

/* Output Data Rates (ODR):
 * -Accelerometers: 1600Hz
 * -Gyroscope:      760Hz
 * -Magnetometer:   100Hz (approximate)
 * -Temperature:      1Hz (approximate)
 * Data is fetched from the accelerometers, gyroscope, and magnetometer at
 * a 100Hz rate. The number of data points in each packet may vary by a small
 * amount. Temperature is read using a housekeeping task in slot 1.
 * 
 * Both the accelerometers and gyroscope have built in FIFOs which buffer their
 * output between reads, ensuring that no data is lost. Each data point from
 * the accelerometer or gyro is 1/ODR later than the one before it. The
 * magnetometer does not have a FIFO, so its data points may not be precisely
 * spaced in time.
 *
 * Because the IMU task downloads data every 10ms, two buffers are maintained
 * to allow the main task to access fetched data. If fresh data is available,
 * get_buf_imu() will return a pointer to the buffer. The data should be copied
 * out of the buffer as quickly as possible. Once data has been copied,
 * release_buf_imu() must be called. Data must be copied out of the buffer within
 * 10ms, or an overrun will occur.
 * TODO: Data range (2G, 4G, etc.).
 */

/* NOTE: The 'IMU' in the error flag names refers to the data retrieval task
 * as a whole, not just the LSM9DS0TR IMU chip. */

/* If true, data loss occurred because a data fetch started while the last
 * buffer was in use. */
extern bool BufferOverrunIMU;

/* If true, data loss occurred due to an overrun in an internal FIFO. */
extern bool FIFOOverrunIMU;

/* If true, data loss occurred due to a bus error.
 * TODO: Develop mitigation strategies (such as retrying the transfer). */
extern bool BusErrorIMU;

/* If true, data loss occurred due to the data fetch not completing in time. */
extern bool BusTimeoutIMU;

/* Current Temperature of LSM9DS0TR IMU. This is the raw value from the device.
 * If no data is available, it will be set to 0xFFFF. */
extern int16_t IMUTemperature;

/* Priority of data stream task interrupt. This is the true numeric value,
 * not the hardware-specific shifted one. This must be higher in value
 * (less urgent) than I2C_IRQ_PRIORITY. */
#define IMU_IRQ_PRIORITY 5

/* IMU temperature sensing housekeeping task slot. */
#define IMU_TEMP_HK_SLOT 1

/* TODO: Add a check to make sure these aren't exceeded. */
/* The maximum number of IMU acceleration data points in a fetch. */
#define MAX_IMU_ACCEL_POINTS 20
/* The maximum number of aux. acceleration data points in a fetch. */
#define MAX_AUX_ACCEL_POINTS 20
/* The maximum number of IMU angular rate data points in a fetch. */
#define MAX_IMU_GYRO_POINTS  10

/* 3-dimensional 16-bit vector. Used to store data points from the accelerometer,
 * gyroscope, and magnetometer. */
typedef struct __attribute__ ((__packed__)) vec3_t {
	int16_t x, y, z;
} vec3_t;

/* Packet data structure. */
typedef struct imu_data_t {
	/* NOTE: Data points with lower indices occurred earlier in time.
	 * (i.e. Data point n occurred before point n+1). */
	uint8_t num_accel; /* Number of IMU acceleration data points. */
	vec3_t accel[MAX_IMU_ACCEL_POINTS]; /* IMU acceleration data. */

	uint8_t num_gyro; /* Number of IMU angular rate data points. */
	vec3_t gyro[MAX_IMU_GYRO_POINTS]; /* IMU angular rate data. */

	uint8_t num_aux; /* Number of aux. accelerometer data points. */
	vec3_t aux[MAX_AUX_ACCEL_POINTS]; /* Aux. accelerometer data. */

	vec3_t mag; /* Magnetometer reading at the time of the fetch. */
	uint32_t mag_time; /* Value of SystemTime at completion of magnetometer
	                    * reading. */
	bool tip_pressed; /* If true, the pen's tip switch was pressed.
	                   * This value is sampled at mag_time. */
	int16_t temperature; /* Approximate temperature when data was collected.
	                      * This is the raw value from the IMU. */
	/* TODO: Verify that temperature changes slowly enough to be adequately
	 * processed by the housekeeping task. */
} imu_data_t;

/* Returns a pointer to an IMU data packet if fresh data is available.
 * Otherwise, NULL is returned.
 * NOTE: Copy data out of the buffer as quickly as
 * possible and call release_buf_imu() when done! */
imu_data_t *get_buf_imu(void);

/* Tell the IMU stream task that data has been copied out of the buffer
 * provided by get_buf_imu(). */
void release_buf_imu(void);

/* Initialize the LSM9DS0TR IMU. start_imu() must be called after this function
 * to synchronize the IMU and aux. accelerometer and start the data streaming task.
 * Returns 0 on success, -1 on error. */
int init_imu(void);

/* Initialize the LIS3DSHTR auxiliary accelerometer. start_imu() must be called
 * after this function to synchronize the IMU and aux. accelerometer and start
 * the data streaming task.
 * Returns 0 on success, -1 on error. */
int init_aux_accel(void);

/* Synchronize the IMU and aux. accelerometer and start the data streaming
 * and temperature update tasks. */
void start_imu(void);

/* Shutdown the IMU data streaming task and put the IMU and aux. accelerometer
 * in a low-power state. */
void shutdown_imu(void);

#endif