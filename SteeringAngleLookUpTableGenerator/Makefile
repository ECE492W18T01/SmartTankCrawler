PROG_NAME = ssr
OBJ = sliplessSteeringRatios_45R_17W_30L.h testCProgram.c
CFLAGS = gcc -std=c99 -Wall

$(PROG_NAME): $(OBJ)
	$(CFLAGS) -o $@ $(OBJ)

clean:
	rm $(PROG_NAME)