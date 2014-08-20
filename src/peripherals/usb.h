/* USB subsystem.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * NoteOn Contains code from the libopencm3 and newlib projects.              */
#ifndef USB_H
#define USH_H
#include <libopencm3/stm32/gpio.h>

/* Initialize the USB interface.
 * Right now, this only sets up the GPIO pin to detect the presence of
 * USB power. */
void init_usb(void);

/* Returns 1 if USB cable is plugged in, 0 otherwise.
 * Call init_usb() before using this macro. */
#define usb_powered() ((GPIOA_IDR & GPIO8)?1:0)

#endif