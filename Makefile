# Smart Pen firmware Makefile
# Options:
CFLAGS:=

# Startup code and linker script.
# Change these if targeting a different microcontroller
STARTUP:=


# Toolchain commands:
CC:= arm-none-eabi-gcc
LD:= arm-none-eabi-ld
OBJCOPY:=arm-none-eabi-objcopy
MAKE:=make

# Used to suppress printing of command
Q := @

.PHONY: all lib build/flash.hex
all: lib build/flash.hex

lib:
	$(Q)if [ ! "`ls -A libopencm3`" ] ; then \
		printf "######## ERROR ########\n"; \
		printf "\tlibopencm3 is not initialized.\n"; \
		printf "\tPlease run:\n"; \
		printf "\t$$ git submodule init\n"; \
		printf "\t$$ git submodule update\n"; \
		printf "\tbefore running make.\n"; \
		printf "\tNote that spaces in the current directory\n"; \
		printf "\tpath will prevent libopencm3 from building.\n"; \
		printf "######## ERROR ########\n"; \
		exit 1; \
		fi
	$(Q)$(MAKE) -C libopencm3