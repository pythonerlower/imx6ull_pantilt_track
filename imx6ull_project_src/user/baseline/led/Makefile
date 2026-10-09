CC ?= gcc
CFLAGS ?= -O2 -Wall

all: led_app

led_app: ledApp.c
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f led_app
