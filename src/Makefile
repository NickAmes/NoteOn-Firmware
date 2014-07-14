# Output file (without extension)
OUTFILE:=../build/smartpen

all: main.o
	$(Q)$(LD) $(LDFLAGS)  *.o ../$(LIBOPENCM3).a -T ../$(LDSCRIPT) -o $(OUTFILE).elf
	$(Q)$(OBJCOPY) -O ihex $(OUTFILE).elf $(OUTFILE).hex

main.o: main.c
	$(Q)$(CC) -c main.c $(INCLUDE) $(ARCH_FLAGS) $(CPPFLAGS) $(CFLAGS)

clean:
	rm -f *.o
	rm -f $(OUTFILE).elf $(OUTFILE).hex
