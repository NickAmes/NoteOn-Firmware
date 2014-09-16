OUTFILE:=../build/smartpen
OUTDIR:=../build/
OBJFILES:= peripherals/usart.o peripherals/newlib_stubs.o peripherals/clock.o  peripherals/systick.o peripherals/i2c.o 
OBJFILES+= peripherals/housekeeping.o peripherals/usb.o peripherals/spi.o board/switches.o board/led.o board/battery.o
OBJFILES+= board/memory.o board/imu.o board/bluetooth.o

all: main.c $(OBJFILES)
	$(Q)mkdir -p ../build/ 
	$(Q)$(CC) main.c $(OBJFILES) ../$(LIBOPENCM3).a -T ../$(LDSCRIPT) -o $(OUTFILE).elf $(INCLUDE) $(ARCH_FLAGS) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS)
	$(Q)$(OBJCOPY) -O ihex $(OUTFILE).elf $(OUTFILE).hex
	$(Q)$(SIZE) $(OUTFILE).elf
	$(Q)$(STM32FLASH) -w $(OUTFILE).hex -g 0x0 -v $(FLASHTTY)

peripherals/usart.o: peripherals/usart.c peripherals/usart.h
	$(Q)$(CC) -c peripherals/usart.c -o peripherals/usart.o $(INCLUDE) $(ARCH_FLAGS) $(CPPFLAGS) $(CFLAGS)

peripherals/newlib_stubs.o: peripherals/newlib_stubs.c
	$(Q)$(CC) -c peripherals/newlib_stubs.c -o peripherals/newlib_stubs.o $(INCLUDE) $(ARCH_FLAGS) $(CPPFLAGS) $(CFLAGS)

peripherals/clock.o: peripherals/clock.c peripherals/clock.h
	$(Q)$(CC) -c peripherals/clock.c -o peripherals/clock.o $(INCLUDE) $(ARCH_FLAGS) $(CPPFLAGS) $(CFLAGS)

board/led.o: board/led.c board/led.h
	$(Q)$(CC) -c board/led.c -o board/led.o $(INCLUDE) $(ARCH_FLAGS) $(CPPFLAGS) $(CFLAGS)

board/switches.o: board/switches.c board/switches.h
	$(Q)$(CC) -c board/switches.c -o board/switches.o $(INCLUDE) $(ARCH_FLAGS) $(CPPFLAGS) $(CFLAGS)

peripherals/systick.o: peripherals/systick.c peripherals/systick.h
	$(Q)$(CC) -c peripherals/systick.c -o peripherals/systick.o $(INCLUDE) $(ARCH_FLAGS) $(CPPFLAGS) $(CFLAGS)

peripherals/i2c.o: peripherals/i2c.c peripherals/i2c.h peripherals/basepri.h
	$(Q)$(CC) -c peripherals/i2c.c -o peripherals/i2c.o $(INCLUDE) $(ARCH_FLAGS) $(CPPFLAGS) $(CFLAGS)

board/battery.o: board/battery.c board/battery.h
	$(Q)$(CC) -c board/battery.c -o board/battery.o $(INCLUDE) $(ARCH_FLAGS) $(CPPFLAGS) $(CFLAGS)

peripherals/housekeeping.o: peripherals/housekeeping.c peripherals/housekeeping.h
	$(Q)$(CC) -c peripherals/housekeeping.c -o peripherals/housekeeping.o $(INCLUDE) $(ARCH_FLAGS) $(CPPFLAGS) $(CFLAGS)

peripherals/usb.o: peripherals/usb.c peripherals/usb.h
	$(Q)$(CC) -c peripherals/usb.c -o peripherals/usb.o $(INCLUDE) $(ARCH_FLAGS) $(CPPFLAGS) $(CFLAGS)

peripherals/spi.o: peripherals/spi.c peripherals/spi.h
	$(Q)$(CC) -c peripherals/spi.c -o peripherals/spi.o $(INCLUDE) $(ARCH_FLAGS) $(CPPFLAGS) $(CFLAGS)

board/memory.o: board/memory.c board/memory.h
	$(Q)$(CC) -c board/memory.c -o board/memory.o $(INCLUDE) $(ARCH_FLAGS) $(CPPFLAGS) $(CFLAGS)

board/imu.o: board/imu.c board/imu.h
	$(Q)$(CC) -c board/imu.c -o board/imu.o $(INCLUDE) $(ARCH_FLAGS) $(CPPFLAGS) $(CFLAGS)

board/bluetooth.o: board/bluetooth.c board/bluetooth.h
	$(Q)$(CC) -c board/bluetooth.c -o board/bluetooth.o $(INCLUDE) $(ARCH_FLAGS) $(CPPFLAGS) $(CFLAGS)

clean:
	rm -f *.o peripherals/*.o board/*.o
	rm -f $(OUTFILE).elf $(OUTFILE).hex
