# stm32f103c8t6
#

PROFILE ?= release

DOC_DIR = doc
DOC_FORMAT = html

PREFIX		?= arm-none-eabi-

DEBUG_FLAGS ?= -std=gnu17 -ggdb3 -O0
STANDARD_FLAGS ?= -std=gnu17 -Os
BUILD_DIR := ./build
SRC_DIR := ./src
INC_DIRS := ./include
SOURCES := abstractSTM32.c
SOURCES += abstractLCD.c
SOURCES += abstractINIT.c
SOURCES += abstractADC.c
SOURCES += abstractDMA.c
SOURCES += abstractUSART.c
SOURCES += abstractLOG.c
SOURCES += abstractENCODER.c
SOURCES += abst_libopencm3.c

CC		= $(PREFIX)gcc
AR		= $(PREFIX)ar

LIB_DIR ?= lib

TARGETS ?= stm32f1 stm32f4

MAKE_FILE_PREFIX = make_
# Do not print "Entering directory ...".
MAKEFLAGS = --no-print-directory

FIFO_DATA_TYPE = uint16_t

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
endif

LIB_MAKES:=$(addprefix $(MAKE_FILE_PREFIX),$(TARGETS))

# Platform specific variables
include $(LIB_MAKES)

# Compiling and linling library
include Makefile.include

all: |  $(BUILD_DIR)/libopencm3.a \
		$(BUILD_DIR)/liblist.a \
		$(BUILD_DIR)/libfifo.a \
		$(BUILD_DIR)/$(ABST_LIBNAME).a

shared:
	$(MAKE) -f $(LIB_MAKES) PREFIX="$(PREFIX)" PROFILE=$(PROFILE) shared

clean:
	$(Q)rm -fr $(BUILD_DIR)
	$(Q)cd $(DOC_DIR) && $(MAKE) clean

documentation:
	$(Q)cd $(DOC_DIR) && $(MAKE) $(DOC_FORMAT)
	$(Q)ln -sf build/html/index.html $(DOC_DIR)/abstractSTM32.html

.PHONY: clean build shared documentation tidy
