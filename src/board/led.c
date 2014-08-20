/* LED control.
 * These macros turn the LED on the top of the pen on and off.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * NoteOn Contains code from the libopencm3 and newlib projects.              */
#include "led.h"
#include <libopencm3/stm32/rcc.h>

/* Setup the GPIO pin for the LED. */
void init_led(void){
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);
	led_off();
}