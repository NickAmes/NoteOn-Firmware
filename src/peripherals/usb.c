/* USB subsystem.
 * 
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 project.                                 */
#include "usb.h"
#include <libopencm3/stm32/rcc.h>

/* Initialize the USB interface.
 * Right now, this only sets up the GPIO pin to detect the presence of
 * USB power. */
void init_usb(void){
	rcc_periph_clock_enable(RCC_GPIOA);
	/* Enable USB power detection pin. */
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO8);
}