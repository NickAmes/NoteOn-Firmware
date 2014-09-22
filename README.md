NoteON Smartpen Firmware
========================
Part of the NoteOn Smartpen Project - http://hackaday.io/project/2678-NoteOn-Smartpen
Copyright 2014 Nick Ames <nick@fetchmodus.org>
Licensed under the GNU GPLv3. See LICENSE file.

Current Status
--------------
Drivers for all peripherals except the Bluetooth transceiver have been completed.

Required Software
-----------------
- libopencm3 (included as git module) https://github.com/libopencm3/libopencm3
- gcc-arm-embedded: https://launchpad.net/gcc-arm-embedded
- STM32Flash (patched for STM32F302x8): https://github.com/NickAmes/STM32Flash
  (dfu-util may also work, but I haven't tested it.)

Source File Layout
------------------
The src/ directory contains the firmware source files. src/peripherals/ contains
modules providing high-level interfaces to on-chip peripherals, such as
I2C and SPI. src/board/ contains modules providing high-level interfaces to 
other ICs on the PCB, such as the IMU and bluetooth controller.

The ld/ directory contains a linker script for the STM32F302x8.

The libopencm3/ directory contains libopencm3 as a git submodule.

The build/ directory contains the compiled binary files.

License
-------
The NoteOn Smartpen firmware is licensed under the GNU GPLv3. NoteOn contains
code from the libopencm3 and newlib projects.

Other components of NoteOn are licensed under
Creative Commons Attribution-ShareAlike 4.0 International.