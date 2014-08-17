/* Housekeeping service. Periodically performs low-priority tasks like updating
 * the battery voltage. Uses timer 6 to periodically call the housekeeping task.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 and newlib projects.                    */
#ifndef HOUSEKEEPING_H
#define HOUSEKEEPING_H

/* The housekeeping task runs every HOUSEKEEPING_PERIOD_MS milliseconds. When it
 * runs, it calls all functions registered in HousekeepingTasks. Each slot in
 * HousekeepingTasks is assigned to a peripheral driver, which places a pointer in
 * its slot to have a function called periodically. NULL
 * slots are ignored. */

/* Housekeeping task period, in milliseconds (ms). */
#define HOUSEKEEPING_PERIOD_MS 500

/* Housekeeping task priority. This should be a large value (low urgency). */
#define HOUSEKEEPING_IRQ_PRIORITY 15

/* Number of housekeeping tasks. */
#define NUM_HOUSEKEEPING_TASKS 5

/* Housekeeping task slots.
 * Allocations:
 *   -Slot 0: battery.h (battery voltage)
 *   -Slot 1: imu.h (IMU temperature)
 */
extern void (*volatile HousekeepingTasks[NUM_HOUSEKEEPING_TASKS])(void);

/* Initialize the housekeeping service. This must be called before using any
 * drivers that use the housekeeping service.
 * This function depends on the current system clock. It should be called
 * whenever the system clock changes. */
void init_housekeeping(void);

#endif