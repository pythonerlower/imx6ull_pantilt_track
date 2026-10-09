CC ?= gcc
CFLAGS ?= -O2 -Wall

all: icm20608_ref62_app

icm20608_ref62_app: icm20608_ref62_app.c
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f icm20608_ref62_app
