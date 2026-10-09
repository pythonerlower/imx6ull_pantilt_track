# Makefile for sensor_alert_app
# Build for ARM

ARCH ?= arm
CROSS_COMPILE ?= arm-linux-gnueabihf-
CC = $(CROSS_COMPILE)gcc
CFLAGS = -Wall -O2 -std=c11 -pthread
LDFLAGS = -pthread -lm

TARGET = sensor_alert_app
SRC = sensor_alert_app.c
UAPI_DIR = ../../kernel/mx6hub

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -I$(UAPI_DIR) -o $@ $< $(LDFLAGS)

clean:
	rm -f $(TARGET)

.PHONY: all clean
