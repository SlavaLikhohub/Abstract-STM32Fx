BUILD_DIR := ./build
PREFIX		?= arm-none-eabi-

TARGETS := stm32f4 # Now only F4, but at least the F1 version is planned as well
MAKE_FILE_PREFIX = make_
# Do not print "Entering directory ...".
MAKEFLAGS = --no-print-directory

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
endif

CC		= $(PREFIX)gcc
AR		= $(PREFIX)ar

all: build


LIB_MAKES:=$(addprefix $(MAKE_FILE_PREFIX),$(TARGETS))
build:
	@echo call $(LIB_MAKES) Makefiles
	$(MAKE) -f $(LIB_MAKES) PREFIX="$(PREFIX)"

clean:
	$(Q)rm -fr $(BUILD_DIR)

.PHONY: clean build
