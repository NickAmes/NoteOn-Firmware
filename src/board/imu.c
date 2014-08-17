/* LSM9DS0TR 9-axis IMU driver.
 * This file provides a high-level interface to the IMU, including a timer-based
 * task that automatically streams data from the sensor.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 */
#include "imu.h"

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
imu_data_t *get_buf_imu(void);

/* Tell the IMU stream task that data has been copied out of the buffer
 * provided by get_buf_imu(). */
void release_buf_imu(void);

/* Initialize the IMU and start the data streaming task.
 * Returns 0 on success, -1 on error.
 * This should be called before initializing the battery gas gauge to reduce
 * traffic on the I2C bus during accelerometer-gyroscope synchronization. */
int init_imu(void);

/* Shutdown the IMU data streaming task and put the IMU in a low-power state. */
void shutdown_imu(void);