PROG_NAME = ssr
OBJ = slipLessSteeringRatios.c slipLessSteeringRatios.h testCProgram.c
CFLAGS = gcc -std=c99 -Wall

$(PROG_NAME): $(OBJ)
	$(CFLAGS) -o $@ $(OBJ)

clean:
	rm $(PROG_NAME)