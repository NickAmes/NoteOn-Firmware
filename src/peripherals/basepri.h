/* ARM Cortex M4 Base Priority Register Set/Get Functions.
 * (For some reason, libopemcm3 doesn't include them).
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * NoteOn Contains code from the libopencm3 and newlib projects.              */

/* Returns the current value of the Base Priority register.
 * This function returns the true numeric value of the priority
 * (0-15) rather than the hardware shifted value. */
static inline __attribute__((always_inline))
uint32_t get_basepri(void)
{
	uint32_t BASEPRI;
	asm("mrs %0, basepri" : "=r" (BASEPRI) : );
	return BASEPRI >> 4;
}


/* Sets the Base Priority register.
 * This function uses the true numeric value of the priority
 * (0-15) rather than the hardware shifted value. */
static inline __attribute__((always_inline))
void set_basepri(uint32_t BASEPRI)
{
	asm("msr basepri, %0" : : "r" (BASEPRI << 4));
}