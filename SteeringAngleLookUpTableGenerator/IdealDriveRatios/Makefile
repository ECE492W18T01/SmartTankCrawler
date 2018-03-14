PROG_NAME = iwr
OBJ = motorDriveR.c motorDriveR.h testCIdealProgram.c
CFLAGS = gcc -std=c99 -Wall

$(PROG_NAME): $(OBJ)
	$(CFLAGS) -o $@ $(OBJ)

clean:
	rm $(PROG_NAME)