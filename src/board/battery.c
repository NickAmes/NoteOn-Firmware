/* STC3115 li-ion battery gas-gauge driver.
 * This file provides a service that periodically measures the voltage of the
 * smart pen battery.
 *
 * Battery voltage updates are triggered by the housekeeping service on slot 0.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 and newlib projects.                    */
#include "battery.h"
#include "../peripherals/housekeeping.h"
#include "../peripherals/i2c.h"

#define STC3115_ADDR 0x70 /* 7-bit I2C address of STC3115. */

/* The current battery voltage in millivolts (mV).
 * This variable is updated periodically with new readings.
 * If an error occurs, or if no data is available, this will be 0xFFFF. */
volatile uint16_t BatteryVoltage = 0xFFFF;

/* Start the battery monitoring task.
 * Returns 0 on success, -1 on error. */
int init_battery(void){
	/* Check that the STC3115 responds over I2C. */
	volatile uint8_t id, flag;
	i2c_ticket_t ticket;
	ticket.rw = I2C_READ;
	ticket.addr = STC3115_ADDR;
	ticket.reg = 24; /* REG_ID */
	ticket.size = 1;
	ticket.data = &id;
	ticket.done_flag = &flag;
	ticket.at_time = 0;
	add_ticket_i2c(&ticket);
	while(0 == flag){
		/* Wait for i2c transaction to complete. */
	}
	if(I2C_DONE == flag){
		if(0x14 == id){
			/* The ID register contains the correct value. */
			HousekeepingTasks[0] = &update_battery_voltage;
			return 0;
		} else {
			return -1;
		}
	} else {
		/* I2C Error. */
		return -1;
	}
}
	

/* Stop the battery monitoring task and put the gas gauge IC into a low-power
 * state. */
void shutdown_battery(void);

/* Battery voltage update task. This is called automatically by the housekeeping
 * service. */
void update_battery_voltage(void){

}
