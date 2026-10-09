CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra -std=gnu99
LDLIBS ?= -pthread

TARGET := hub_monitor
SRC := hub_monitor.c

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $< $(LDLIBS)

clean:
	rm -f $(TARGET)
