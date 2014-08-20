/* STC3115 li-ion battery gas-gauge driver.
 * This file provides a service that periodically measures the voltage of the
 * smart pen battery.
 *
 * Battery voltage updates are triggered by the housekeeping service on slot 0.
 * 
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * NoteOn Contains code from the libopencm3 and newlib projects.              */
#ifndef BATTERY_H
#define BATTERY_H
#include <stdint.h>

/* Battery voltage service housekeeping task slot. */
#define BATTERY_HK_SLOT 0

/* The current battery voltage in millivolts (mV).
 * This variable is updated periodically with new readings.
 * If an error occurs, or if no data is available, this will be 0xFFFF. */
extern volatile uint16_t BatteryVoltage;

/* Start the battery monitoring task.
 * Returns 0 on success, -1 on error. */
int init_battery(void);

/* Stop the battery monitoring task and put the gas gauge IC into a low-power
 * state. */
void shutdown_battery(void);

/* Battery voltage update task. This is called automatically by the housekeeping
 * service. */
void update_battery_voltage(void);


#endif
