/* LED control.
 * These macros turn the LED on the top of the pen on and off.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 project.                                 */
#ifndef LED_H
#define LED_H
#include <libopencm3/stm32/gpio.h>

/* Setup the GPIO pin for the LED. */
void init_led(void);

/* Turn the LED on. */
#define led_on() (GPIOA_BSRR = GPIO13)

/* Turn the led off. */
#define led_off() (GPIOA_BSRR = (GPIO13 << 16))

/* Toggle the led. */
#define led_toggle() (GPIO_BSRR = (GPIOA_ODR & GPIO13)?(GPIO13 << 16):(GPIO13))

#endif