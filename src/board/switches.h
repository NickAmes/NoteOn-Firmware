/* Functions for the top switch and tip switch.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 project.                                 */
#ifndef SWITCHES_H
#define SWITCHES_H
#include <libopencm3/stm32/gpio.h>

/* Setup GPIOs for the top switch and tip switch. */
void init_switches(void);

/* Evaluates to 1 if the top switch is pressed, 0 otherwise. */
#define top_switch_pressed() (!(GPIOA_IDR & GPIO14))

/* Evaluates 1 if the tip switch is pressed, 0 otherwise. */
#define tip_switch_pressed() (!(GPIOA_IDR & GPIO3))



#endif