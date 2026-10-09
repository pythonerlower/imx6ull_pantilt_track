CC ?= arm-linux-gnueabihf-gcc

all: ap3216c_myself_app

ap3216c_myself_app: ap3216c_myself_app.c
	$(CC) -Wall -O2 -o $@ $<

clean:
	rm -f ap3216c_myself_app *.o

install: ap3216c_myself_app
	scp ap3216c_myself_app root@192.168.137.50:/home/root/

.PHONY: all clean install
