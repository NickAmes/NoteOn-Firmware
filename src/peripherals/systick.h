/* System time functions.
 * 
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 and newlib projects.                    */
#ifndef SYSTICK_H
#define SYSTICK_H
#include <stdint.h>

/* Setup the SYSTICK timer with rollover set to the given period. The systick
 * clock is the system clock. */
void init_systick(uint32_t period);

#endif