OUTFILE:=../build/smartpen

OBJFILES:= peripherals/usart.o peripherals/newlib_stubs.o peripherals/clock.o peripherals/led.o peripherals/systick.o peripherals/i2c.o

all: main.c $(OBJFILES)
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

peripherals/led.o: peripherals/led.c peripherals/led.h
	$(Q)$(CC) -c peripherals/led.c -o peripherals/led.o $(INCLUDE) $(ARCH_FLAGS) $(CPPFLAGS) $(CFLAGS)

peripherals/systick.o: peripherals/systick.c peripherals/systick.h
	$(Q)$(CC) -c peripherals/systick.c -o peripherals/systick.o $(INCLUDE) $(ARCH_FLAGS) $(CPPFLAGS) $(CFLAGS)

peripherals/i2c.o: peripherals/i2c.c peripherals/i2c.h
	$(Q)$(CC) -c peripherals/i2c.c -o peripherals/i2c.o $(INCLUDE) $(ARCH_FLAGS) $(CPPFLAGS) $(CFLAGS)
	
clean:
	rm -f *.o
	rm -f peripherals/*.o
	rm -f $(OUTFILE).elf $(OUTFILE).hex
