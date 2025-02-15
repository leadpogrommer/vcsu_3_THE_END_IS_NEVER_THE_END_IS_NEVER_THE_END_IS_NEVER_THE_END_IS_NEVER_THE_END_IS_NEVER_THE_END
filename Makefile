DEVICE          = stm32f411ceu6
OPENCM3_DIR     = thirdparty/libopencm3
OBJS            += main.o
CFLAGS 			+= -nostartfiles -fno-common -g -Og
LDFLAGS 		+= -nostartfiles -fno-common

# Toolchain
CC=arm-none-eabi-gcc
CXX=arm-none-eabi-g++
LD=arm-none-eabi-ld
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump

include $(OPENCM3_DIR)/mk/genlink-config.mk
include $(OPENCM3_DIR)/mk/gcc-config.mk

.PHONY: clean all

all: binary.elf binary.hex

clean:
	$(Q)$(RM) -f binary.* *.o


include $(OPENCM3_DIR)/mk/genlink-rules.mk
include $(OPENCM3_DIR)/mk/gcc-rules.mk

$(LIBDEPS): # TODO: костыль
	$(MAKE) -C $(OPENCM3_DIR) TARGETS='stm32/f4'