# Klipper build system
#
# Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# Output directory
OUT=out/

# Kconfig includes
export KCONFIG_CONFIG     := $(CURDIR)/.config
-include $(KCONFIG_CONFIG)

# Common command definitions moved after KCONFIG_CONFIG include
ifeq ($(CONFIG_MACH_AVR),y)
CC:=avr-gcc
AS:=avr-as
LD:=avr-ld
OBJCOPY:=avr-objcopy
OBJDUMP:=avr-objdump
STRIP:=avr-strip
else
# Default/Host compiler if not AVR (or other cross-compile targets)
CC:=gcc
AS:=as
LD:=ld
OBJCOPY:=objcopy
OBJDUMP:=objdump
STRIP:=strip
endif
CPP:=cpp
PYTHON:=python3

# Source files
src-y =
dirs-y = src

# Default compiler flags
cc-option=$(shell if test -z "`$(1) $(2) -S -o /dev/null -xc /dev/null 2>&1`" \
    ; then echo "$(2)"; else echo "$(3)"; fi ;)

CFLAGS := -iquote $(OUT) -iquote src -iquote $(OUT)board-generic/ \
		-std=gnu11 -O2 -MD -Wall \
		-Wold-style-definition $(call cc-option,$(CC),-Wtype-limits,) \
    -ffunction-sections -fdata-sections -fno-delete-null-pointer-checks
CFLAGS += -flto=auto -fwhole-program -fno-use-linker-plugin -ggdb3

ifeq ($(CONFIG_MACH_AVR),y)
MCU_NAME_FOR_MAKE := $(patsubst "%",%,$(CONFIG_MCU))
CFLAGS += -mmcu=$(MCU_NAME_FOR_MAKE) -DF_CPU=$(CONFIG_CLOCK_FREQ)
endif

OBJS_klipper.elf = $(patsubst %.c, $(OUT)src/%.o,$(sort $(src-y)))
OBJS_klipper.elf += $(OUT)compile_time_request.o
CFLAGS_klipper.elf = $(CFLAGS) -Wl,--gc-sections

CPPFLAGS = -I$(OUT) -P -MD -MT $@

# Default targets
target-y := $(OUT)klipper.elf

all:

# Run with "make V=1" to see the actual compile commands
ifdef V
Q=
else
Q=@
MAKEFLAGS += --no-print-directory
endif

# Include board specific makefile
include src/Makefile
-include src/$(patsubst "%",%,$(CONFIG_BOARD_DIRECTORY))/Makefile

# Explicitly add linux/main.c if CONFIG_MACH_LINUX is set, to ensure main() is found
# This is a diagnostic step / workaround if src/linux/Makefile isn't populating src-y correctly.
ifeq ($(CONFIG_MACH_LINUX),y)
src-y += linux/main.c linux/timer.c linux/console.c linux/watchdog.c \
         linux/pca9685.c linux/spidev.c linux/analog.c linux/hard_pwm.c \
         linux/i2c.c linux/gpio.c generic/crc16_ccitt.c generic/alloc.c \
         linux/sensor_ds18b20.c
endif

# AVR specific sources
ifeq ($(CONFIG_MACH_AVR),y)
src-y += avr/main.c avr/timer.c avr/gpio.c avr/adc.c avr/spi.c avr/i2c.c \
         avr/hard_pwm.c avr/watchdog.c avr/serial.c generic/serial_irq.c
src-y += $(if $(filter y,$(CONFIG_USBSTD_SERIAL)),avr/usbserial.c)
endif

# Workaround for CONFIG_HAVE_GPIO based sources
src-y += $(if $(filter y,$(CONFIG_HAVE_GPIO)),initial_pins.c gpiocmds.c stepper.c endstop.c trsync.c)

################ Main build rules

$(OUT)%.o: %.c $(OUT)autoconf.h
	@echo "  Compiling $@"
	$(Q)mkdir -p $(dir $@)
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

$(OUT)%.ld: %.lds.S $(OUT)autoconf.h
	@echo "  Preprocessing $@"
	$(Q)$(CPP) -I$(OUT) -P -MD -MT $@ $< -o $@

$(OUT)klipper.elf: $(OBJS_klipper.elf)
	@echo "  Linking $@"
	$(Q)$(CC) $(OBJS_klipper.elf) $(CFLAGS_klipper.elf) -o $@
	$(Q)scripts/check-gcc.sh $@ $(OUT)compile_time_request.o

################ Compile time requests

$(OUT)%.o.ctr: $(OUT)%.o
	$(Q)$(OBJCOPY) -j '.compile_time_request' -O binary $^ $@

$(OUT)compile_time_request.o: $(patsubst %.c, $(OUT)src/%.o.ctr,$(src-y)) ./scripts/buildcommands.py
	@echo "  Building $@"
	$(Q)cat $(patsubst %.c, $(OUT)src/%.o.ctr,$(sort $(src-y))) | tr -s '\0' '\n' > $(OUT)compile_time_request.txt
	$(Q)$(PYTHON) ./scripts/buildcommands.py -d klippy/chelper/atmega2560.dict -t "$(CC);$(AS);$(LD);$(OBJCOPY);$(OBJDUMP);$(STRIP)" $(OUT)compile_time_request.txt $(OUT)compile_time_request.c
	$(Q)$(CC) $(CFLAGS) -c $(OUT)compile_time_request.c -o $@

################ Auto generation of "board/" include file link

create-board-link:
	@echo "  Creating symbolic link $(OUT)board"
	$(Q)mkdir -p $(addprefix $(OUT), $(dirs-y))
	$(Q)rm -f $(OUT)*.d $(patsubst %,$(OUT)%/*.d,$(dirs-y))
	$(Q)rm -f $(OUT)board
	$(Q)ln -sf $(CURDIR)/src/$(CONFIG_BOARD_DIRECTORY) $(OUT)board
	$(Q)mkdir -p $(OUT)board-generic
	$(Q)rm -f $(OUT)board-generic/board
	$(Q)ln -sf $(CURDIR)/src/generic $(OUT)board-generic/board

# Hack to rebuild OUT directory and reload make dependencies on Kconfig change
$(OUT)board-link: $(KCONFIG_CONFIG)
	$(Q)mkdir -p $(OUT)
	$(Q)echo "# Makefile board-link rule" > $@
	$(Q)$(MAKE) create-board-link
include $(OUT)board-link

################ Kconfig rules

$(OUT)autoconf.h: $(KCONFIG_CONFIG)
	@echo "  Building $@"
	$(Q)mkdir -p $(OUT)
	$(Q) KCONFIG_AUTOHEADER=$@ $(PYTHON) lib/kconfiglib/genconfig.py src/Kconfig

$(KCONFIG_CONFIG) olddefconfig: src/Kconfig
	$(Q)$(PYTHON) lib/kconfiglib/olddefconfig.py src/Kconfig

menuconfig:
	$(Q)$(PYTHON) lib/kconfiglib/menuconfig.py src/Kconfig

################ Generic rules

# Make definitions
.PHONY : all clean distclean olddefconfig menuconfig create-board-link FORCE
.DELETE_ON_ERROR:

all: $(target-y)

clean:
	$(Q)rm -rf $(OUT)

distclean: clean
	$(Q)rm -f .config .config.old

-include $(OUT)*.d $(patsubst %,$(OUT)%/*.d,$(dirs-y))
