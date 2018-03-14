PROG_NAME = ssr
OBJ = sliplessSteeringRatios.c sliplessSteeringRatios.h testCProgram.c
CFLAGS = gcc -std=c99 -Wall

$(PROG_NAME): $(OBJ)
	$(CFLAGS) -o $@ $(OBJ)

clean:
	rm $(PROG_NAME)