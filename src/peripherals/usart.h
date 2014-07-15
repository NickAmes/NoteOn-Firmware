/* High-level USART control functions.
 * This file implements a simple non-buffered serial channel over USART1,
 * including newlib stub functions.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 * Contains code from the libopencm3 project.                                 */
#ifndef USART_H
#define USART_H

/* After initialize the USART with init_usart, use the read() and write()
 * (or stdio functions) to receive and send data.
 * WARNING: stdio functions that read() currently don't work.
 * These functions will block until the data has been transfered. */

/* Baud rate. Data is transfered at the baud rate with 8 data bit, no parity,
 * and 1 stop bit. */
#define USART_BAUD 115200

/* Initialize USART1. This function sets the baud rate based on the current clock
 * frequency. If the clock is changed this function must be called again. */
void init_usart(void);

/* Shutdown USART1 to save power.
 * If write() if called when the USART is shutdown it will behave as if the
 * data was instantly successfully transmitted. If read() is called when the
 * USART is shutdown the USART will be initialized and the read will proceed
 * normally. */
void shutdown_usart(void);

/* Newlib read() stub. */
int _read(int file, char *ptr, int len);

/* Newlib write() stub. */
int _write(int file, char *ptr, int len);

#endif