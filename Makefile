PROFILE ?= release

DOC_DIR = doc
DOC_FORMAT = html

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

all: build


LIB_MAKES:=$(addprefix $(MAKE_FILE_PREFIX),$(TARGETS))
build:
	@echo call $(LIB_MAKES) Makefiles
	$(MAKE) -f $(LIB_MAKES) PREFIX="$(PREFIX)"

shared:
	$(MAKE) -f $(LIB_MAKES) PREFIX="$(PREFIX)" PROFILE=$(PROFILE) shared

clean:
	$(Q)rm -fr $(BUILD_DIR)
	$(Q)cd $(DOC_DIR) && $(MAKE) clean

documentation:
	$(Q)cd $(DOC_DIR) && $(MAKE) $(DOC_FORMAT)
	$(Q)ln -sf build/html/index.html $(DOC_DIR)/abstractSTM32.html

.PHONY: clean build shared documentation
