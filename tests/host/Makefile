CC      ?= cc
CFLAGS  ?= -Wall -Wextra -Werror
TARGET  := /tmp/test_adsb_state

all: run

$(TARGET): test_adsb_state.c ../../main/adsb_state.c
	$(CC) $(CFLAGS) -I ../../main -o $@ ../../main/adsb_state.c test_adsb_state.c

run: $(TARGET)
	$(TARGET)

clean:
	rm -f $(TARGET)

.PHONY: all run clean