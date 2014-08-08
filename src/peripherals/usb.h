/* USB subsystem.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 project.                                 */
#ifndef USB_H
#define USH_H
#include <libopencm3/stm32/gpio.h>

/* For starters, replace the USART with a CDC-ACM usb function. */

/* Initialize the USB interface.
 * TODO: When, prerequisites. */
void init_usb(void);

/* Returns 1 if USB cable is plugged in, 0 otherwise.
 * Call init_usb() before using this macro. */
#define usb_powered() ((GPIOA_IDR & GPIO8)?1:0)

#endif