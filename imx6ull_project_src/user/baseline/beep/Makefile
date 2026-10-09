CC ?= gcc
CFLAGS ?= -O2 -Wall

all: beep_app

beep_app: miscbeepApp.c
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f beep_app
