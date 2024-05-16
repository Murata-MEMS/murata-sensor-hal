
IDIR = -I../android/include

CC=$(CROSS_COMPILE)gcc
LD=$(CROSS_COMPILE)gcc
LIB=-ldl -lpthread
CFLAGS=$(IDIR) -Wall -Werror

CPPFLAGS := $(CFLAGS)

.PHONY: test-murata-hal

all: test-murata-hal

%.o: %.c
	$(CC) -c -o $@ $< $(CFLAGS)

test-murata-hal: test-murata-hal.o
	$(CC) $(CFLAGS) test-murata-hal.o $(LIB) -o $@

clean:
	rm -f *.o test-murata-hal
