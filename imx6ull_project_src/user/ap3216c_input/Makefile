CC ?= gcc
CFLAGS ?= -O2 -Wall

all: ap3216c_input_app

ap3216c_input_app: ap3216c_input_app.c
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f ap3216c_input_app

.PHONY: all clean
