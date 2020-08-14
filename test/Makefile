DEBUG ?= 5
TARGET := test
SRCDIR  = .

SRCS = $(TARGET).cpp
SRCS += ../../src/abstractSTM32.c

LIBS = 
MANUAL_LINK = 

BUILDDIR = ./.build

CFLAGS = -O2 -Wall -Wextra -Wpedantic -std=gnu++2a

# Link boost
CFLAGS += -I/usr/lib/


# OPENCM3 LINKING
OPENCM3_DIR = ../lib/libopencm3

DEVICE ?= stm32f407vgt6
# Definitions required to generate linker script
include $(OPENCM3_DIR)/mk/genlink-config.mk

CFLAGS += $(TGT_CFLAGS) $(genlink_cppflags)

CFLAGS += $(shell pkg-config --cflags $(LIBS))
LDFLAGS = $(shell pkg-config --libs $(LIBS)) -Wl,--as-needed
LDFLAGS += $(MANUAL_LINK)
STATICLINK = /usr/lib/libboost_unit_test_framework.a
CC := g++

CFLAGS += -I../include
CFLAGS += -I$(OPENCM3_DIR)/include

.PHONY: all clean tidy

all: $(TARGET)

$(BUILDDIR)/%.o: $(SRCDIR)/%.cpp | $(BUILDDIR)
	$(CC) $(CFLAGS) $(addprefix -I,$(INCDIRS)) -c $< -o $@

$(BUILDDIR)/$(TARGET): $(addprefix $(BUILDDIR)/,$(SRCS:.cpp=.o))
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@ $(STATICLINK)

$(TARGET): $(BUILDDIR)/$(TARGET)
	ln -sf $< $@

$(BUILDDIR):
	mkdir -p $@

clean:
	-rm -rf $(BUILDDIR)
	-rm -rf $(FUNCBUILDDIR)

tidy: clean
	-rm -f $(TARGET)
