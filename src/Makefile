OUTFILE:=../build/smartpen

OBJFILES:= peripherals/usart.o

all: main.c $(OBJFILES)
	$(Q)$(CC) main.c $(OBJFILES) ../$(LIBOPENCM3).a -T ../$(LDSCRIPT) -o $(OUTFILE).elf $(INCLUDE) $(ARCH_FLAGS) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS)
	$(Q)$(OBJCOPY) -O ihex $(OUTFILE).elf $(OUTFILE).hex
	$(Q)$(SIZE) $(OUTFILE).elf

peripherals/usart.o: peripherals/usart.c peripherals/usart.h
	$(Q)$(CC) -c peripherals/usart.c -o peripherals/usart.o $(INCLUDE) $(ARCH_FLAGS) $(CPPFLAGS) $(CFLAGS)

clean:
	rm -f *.o
	rm -f peripherals/*.o
	rm -f $(OUTFILE).elf $(OUTFILE).hex
