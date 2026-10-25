CC = gcc
CFLAGS = -O2 -Wall -Wextra -std=c99 -D_GNU_SOURCE

test: test_host
	./test_host

test_host: test_host.c steel_shifter.c steel_changer.c steel_shifter.h steel_changer.h
	$(CC) $(CFLAGS) -o $@ test_host.c steel_shifter.c steel_changer.c -lm

clean:
	rm -f test_host
