/* nRF8001 Bluetooth transceiver driver.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * NoteOn Contains code from the libopencm3 and newlib projects.              */
#ifndef BLUETOOTH_H
#define BLUETOOTH_H

/* TODO: Decide on NoteOn bluetooh API */

/* Initialize the nRF8001 Bluetooth transceiver.
 * Returns 0 on success, -1 on error. */
int init_bt(void);

/* Put the Bluetooth transceiver into a low power state. */
void shutdown_bt(void);

#endif