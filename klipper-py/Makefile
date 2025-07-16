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

# Common command definitions
CC=$(CROSS_PREFIX)gcc
AS=$(CROSS_PREFIX)as
LD=$(CROSS_PREFIX)ld
OBJCOPY=$(CROSS_PREFIX)objcopy
OBJDUMP=$(CROSS_PREFIX)objdump
STRIP=$(CROSS_PREFIX)strip
CPP=cpp
PYTHON=python3

# Source files
src-y =
dirs-y = src

# Default compiler flags
cc-option=$(shell if test -z "`$(1) $(2) -S -o /dev/null -xc /dev/null 2>&1`" \
    ; then echo "$(2)"; else echo "$(3)"; fi ;)

# Klipper Rust Port Integration
KLIPPER_RUST_PORT_DIR := $(CURDIR)/klipper_rust_port
# TODO: Dynamically set RUST_TARGET_TRIPLE based on Klipper MCU config
RUST_TARGET_TRIPLE ?= thumbv7m-none-eabi
KLIPPER_RUST_LIB_PATH := $(KLIPPER_RUST_PORT_DIR)/target/$(RUST_TARGET_TRIPLE)/release
KLIPPER_RUST_LIB_NAME := klipper_rust_port
KLIPPER_RUST_STATIC_LIB := $(KLIPPER_RUST_LIB_PATH)/lib$(KLIPPER_RUST_LIB_NAME).a
KLIPPER_RUST_HEADER_DIR := $(KLIPPER_RUST_PORT_DIR)

CFLAGS := -iquote $(OUT) -iquote src -iquote $(OUT)board-generic/ \
		-I$(KLIPPER_RUST_HEADER_DIR) \
		-std=gnu11 -O2 -MD -Wall \
		-Wold-style-definition $(call cc-option,$(CC),-Wtype-limits,) \
    -ffunction-sections -fdata-sections -fno-delete-null-pointer-checks
CFLAGS += -flto=auto -fwhole-program -fno-use-linker-plugin -ggdb3

OBJS_klipper.elf = $(patsubst %.c, $(OUT)src/%.o,$(src-y))
OBJS_klipper.elf += $(OUT)compile_time_request.o
# Add Rust library linker flags
CFLAGS_klipper.elf = $(CFLAGS) -Wl,--gc-sections -L$(KLIPPER_RUST_LIB_PATH) -l$(KLIPPER_RUST_LIB_NAME)

CPPFLAGS = -I$(OUT) -I$(KLIPPER_RUST_HEADER_DIR) -P -MD -MT $@

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

################ Main build rules

$(OUT)%.o: %.c $(OUT)autoconf.h
	@echo "  Compiling $@"
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

$(OUT)%.ld: %.lds.S $(OUT)autoconf.h
	@echo "  Preprocessing $@"
	$(Q)$(CPP) -I$(OUT) -I$(KLIPPER_RUST_HEADER_DIR) -P -MD -MT $@ $< -o $@

# Rule to build the Rust static library
$(KLIPPER_RUST_STATIC_LIB): $(KLIPPER_RUST_PORT_DIR)/src/lib.rs $(KLIPPER_RUST_PORT_DIR)/Cargo.toml $(KLIPPER_RUST_PORT_DIR)/build.rs
	@echo "  Building Rust library $@"
	+$(Q)cd $(KLIPPER_RUST_PORT_DIR) && PATH=$(HOME)/.cargo/bin:$(PATH) cargo build --target=$(RUST_TARGET_TRIPLE) --release

$(OUT)klipper.elf: $(OBJS_klipper.elf) $(KLIPPER_RUST_STATIC_LIB)
	@echo "  Linking $@"
	$(Q)$(CC) $(OBJS_klipper.elf) $(CFLAGS_klipper.elf) -o $@
	$(Q)scripts/check-gcc.sh $@ $(OUT)compile_time_request.o

################ Compile time requests

$(OUT)%.o.ctr: $(OUT)%.o
	$(Q)$(OBJCOPY) -j '.compile_time_request' -O binary $^ $@

$(OUT)compile_time_request.o: $(patsubst %.c, $(OUT)src/%.o.ctr,$(src-y)) ./scripts/buildcommands.py
	@echo "  Building $@"
	$(Q)cat $(patsubst %.c, $(OUT)src/%.o.ctr,$(src-y)) | tr -s '\0' '\n' > $(OUT)compile_time_request.txt
	$(Q)$(PYTHON) ./scripts/buildcommands.py -d $(OUT)klipper.dict -t "$(CC);$(AS);$(LD);$(OBJCOPY);$(OBJDUMP);$(STRIP)" $(OUT)compile_time_request.txt $(OUT)compile_time_request.c
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
