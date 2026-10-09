CC ?= gcc
CFLAGS ?= -O2 -Wall

all: icm20608_app

icm20608_app: icm20608App.c
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f icm20608_app
