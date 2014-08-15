/* External flash memory driver.
 * This file implements a high-level interface to a Micron N25Q512A1
 * 512Mb (64MB) SPI NOR flash memory.
 *
 * This file is a part of the firmware for the NoteOn Smartpen.
 * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
 */
#ifndef MEMORY_H
#define MEMORY_H
#include <stdint.h>

/* Memory organization according to the datasheet:
 * "The memory is configured as 67,108,864 bytes (8 bits each);
 * 1024 sectors (64KB each);
 * 16,384 subsectors (4KB each);
 * and 262,144 pages (256 bytes each)" */

/* These functions block until their data has been transfered to/from the
 * chip. Erasing and programming operations happen internally on the chip,
 * and can take some time.
 * WARNING: Subsequent operations will stall until the internal
 * erasing/programming process is complete. The system should not be reset
 * until these operations have completed, or problems can occur if the
 * next power-up happens while these operations are in progress.
 * Use shutdown_memory() to ensure that this doesn't happen. */

/* Setup required pins and check that the memory chip is responding correctly.
 * Returns 0 on success, -1 on error. */
int init_memory(void);

/* Wait for any internal operations to complete and put the chip into a
 * low-power state. */
void shutdown_memory(void);

/* Read data from memory into the provided buffer. The address is in bytes,
 * and need not be aligned to any boundary. The size is in bytes, and need not
 * be a multiple of 2. */
void read_mem(uint32_t address, uint8_t *data, uint32_t size);

/* Program a page (256 byte block).
 * WARNING: Programming a page can take up to 5 ms to complete.
 * This function will return quickly and the process
 * happens internally in the chip, but subsequent memory operations will
 * stall until programming is finished. */
void program_page_mem(uint32_t page, const uint8_t *data);

/* Erase the specified sector(s). The start and end sector numbers are an
 * inclusive range; the specified sectors and all in between will be erased.
 * Erasing sets all bits to 1.
 * WARNING: Erasing a sector can take up to 3 *seconds* to complete.
 * This function will return quickly and the process
 * happens internally in the chip, but subsequent memory operations will
 * stall until erasing is finished. */
void erase_sector_mem(uint16_t start, uint16_t end);

/* Erase the specified subsectors(s). The start and end subsector numbers are an
 * inclusive range; the specified subsectors and all in between will be erased.
 * Erasing sets all bits to 1.
 * WARNING: Erasing a subsector can take up to 800 ms to complete.
 * This function will return quickly and the process
 * happens internally in the chip, but subsequent memory operations will
 * stall until erasing is finished. */
void erase_subsector_mem(uint16_t start, uint16_t end);

/* Erase one of the chip's two 32MB dies.
 *   -die is either 0 (lower 32MB) or 1 (upper 32MB)
 *   -passcode must be 0xDEAD to proceed.
 * WARNING: Erasing a die can take up to 4 *minutes* to complete.
 * This function will return quickly and the process
 * happens internally in the chip, but subsequent memory operations will
 * stall until erasing is finished. */
void erase_chip_mem(uint8_t die, uint16_t passcode);

#endif