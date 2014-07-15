# Smart Pen firmware Makefile
# Options:
# C Compiler Flags:
CFLAGS=-Os -g -Wextra -Wshadow -Wimplicit-function-declaration -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS+= -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes
CFLAGS+= -fno-common -ffunction-sections -fdata-sections

# C Pre-Processor Flags:
CPPFLAGS= -Wall -Wundef
CPPFLAGS+= -DSTM32F3

# Floating-Point and architecture flags:
FP_FLAGS= -mfloat-abi=hard -mfpu=fpv4-sp-d16
ARCH_FLAGS= -mthumb -mcpu=cortex-m4 $(FP_FLAGS)

# Linker Flags
LDFLAGS= --static -nostartfiles
LDFLAGS+= -Wl,--gc-sections
LDFLAGS+= -lc -lgcc -lnosys

export CFLAGS
export CPPFLAGS
export ARCH_FLAGS
export LDFLAGS

# Linker script and libopencm3 library archive.
# Change if targeting a different microcontroller
export LDSCRIPT:=ld/stm32f302x8.ld
export LIBOPENCM3:=libopencm3/lib/libopencm3_stm32f3

# Include directories:
export INCLUDE:=-I../libopencm3/include/

# Toolchain commands:
export CC:= arm-none-eabi-gcc
export LD:= arm-none-eabi-ld
export OBJCOPY:= arm-none-eabi-objcopy
export OBJDUMP:= arm-none-eabi-objdump
MAKE:= make

# Used to suppress printing of command
Q := @
export Q

.PHONY: all build clean

all: $(LIBOPENCM3).a build

$(LIBOPENCM3).a:
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

build:
	$(Q)$(MAKE) --quiet -C src/

clean:
	$(Q)$(MAKE) --quiet -C src/ clean