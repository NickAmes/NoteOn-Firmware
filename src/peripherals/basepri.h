/* ARM Cortex M4 Base Priority Register Set/Get Functions.
 * (For some reason, libopemcm3 doesn't include them).
 * This file contains code from the core_cmFunc.h file by ARM limited.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 */

/* Returns the current value of the Base Priority register. */
static inline __attribute__((always_inline))
uint32_t __get_BASEPRI(void)
{
	register uint32_t __regBasePri __asm("basepri");
	return(__regBasePri);
}


/* Sets the Base Priority register. */
static inline __attribute__((always_inline))
void __set_BASEPRI(uint32_t basePri)
{
	register uint32_t __regBasePri __asm("basepri");
	__regBasePri = (basePri & 0xff);
	__regBasePri = __regBasePri; /* Suppress unused variable warnings. */
}