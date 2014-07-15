/* System time functions.
 * 
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 and newlib projects.                    */
#ifndef SYSTICK_H
#define SYSTICK_H

/* Setup the SYSTICK timer. */
void init_systick(void);

#endif