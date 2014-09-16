/* nRF8001 Bluetooth transceiver driver.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * NoteOn Contains code from the libopencm3 and newlib projects.              */
#include "bluetooth.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include "../peripherals/spi.h"

/* Evaluates to the current state of the RDYn line. */
#define RDYn()  ((GPIOB_IDR & GPIO0)?1:0)

/* Set the REQn line high or low. */
#define REQn_high() (GPIOA_BSRR = GPIO7)
#define REQn_low() (GPIOA_BSRR = (GPIO7 << 16))

/* Initialize the nRF8001 Bluetooth transceiver.
 * Returns 0 on success, -1 on error. */
int init_bt(void);

/* Put the Bluetooth transceiver into a low power state. */
void shutdown_bt(void);
