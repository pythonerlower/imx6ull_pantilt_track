CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra -std=gnu99

TARGET := v4l2_fb_preview
SRC := v4l2_fb_preview.c

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $<

clean:
	rm -f $(TARGET)
