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
	flag = 0;
	add_ticket_i2c(&ticket);
	while(I2C_BUSY == flag){
		/* Wait for i2c transaction to complete. */
		
	}
	if(I2C_DONE == flag){
		if(0x14 == id){
			/* The ID register contains the correct value. */
			HousekeepingTasks[BATTERY_HK_SLOT] = &update_battery_voltage;
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
void shutdown_battery(void){
	/* The IC goes into sleep mode automatically, so the only thing to do
	 * is stop the update_battery_voltage() task. */
	HousekeepingTasks[BATTERY_HK_SLOT] = 0;
}

/* Battery voltage update task. This is called automatically by the housekeeping
 * service. */
void update_battery_voltage(void){
	/* Due to an error in design revision 1, the gas gauge IC thinks that
	 * a battery isn't present, and can't make multiple measurements of
	 * the battery voltage.
	 * However, the IC still measures the battery voltage on power-up, which
	 * provides a work-around. Every other measurement cycle, the IC is soft-reset.
	 * It then spends 250ms taking a battery voltage measurement, and shuts
	 * down. */
	static uint8_t state; /* 0 = Reset the IC, 1 = fetch the value. */
	/* High voltage byte and low voltage byte ticket flags. */
	static volatile uint8_t volt_H_flag, volt_L_flag;
	/* Reset command byte. */
	static const uint8_t reset_cmd = 0x10; /* Soft reset command. */
	/* High and low voltage bytes. */
	static volatile volt_H, volt_L;
	i2c_ticket_t ticket; /* I2C ticket. */
	ticket.addr = 0x70;
	ticket.at_time = 0;
	ticket.size = 1;

	if(0 == state){
		/* Tell the IC to perform a soft-reset and process the data from
		 * the previous measurement. */
		state = 1;
		ticket.rw = I2C_WRITE;
		ticket.reg = 1; 
		ticket.data = &reset_cmd;
		ticket.done_flag = 0;
		add_ticket_i2c(&ticket);
		
		if(volt_H_flag == I2C_DONE && volt_L_flag == I2C_DONE){
			BatteryVoltage = ((volt_L | (volt_H << 8)) * 22)/10;

		} else {
			BatteryVoltage = 0xFFFF;
		}
	} else {
		/* Fetch the voltage data. */
		volt_L_flag = 0;
		volt_H_flag = 0;
		
		state = 0;
		ticket.rw = I2C_READ;
		
		ticket.reg = 8;
		ticket.data = &volt_L;
		ticket.done_flag = &volt_L_flag;
		add_ticket_i2c(&ticket);

		ticket.reg = 9;
		ticket.data = &volt_H;
		ticket.done_flag = &volt_H_flag;
		add_ticket_i2c(&ticket);
	}
}
