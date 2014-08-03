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

/* Start the battery monitoring task.
 * Returns 0 on success, -1 on error. */
int init_battery(void);
	

/* Stop the battery monitoring task and put the gas gauge IC into a low-power
 * state. */
void shutdown_battery(void); 
