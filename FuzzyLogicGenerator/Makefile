PROG_NAME = fuzzy
OBJ = FuzzyLogicProcessor.h FuzzyLogicProcessor.c sliplessSteeringRatios_45R_17W_30L.h fMatrices_servoFuzzyThree.h
CFLAGS = gcc -std=c99 -Wall
MATH = -lm

$(PROG_NAME): $(OBJ)
	$(CFLAGS) -o $@ $(OBJ) $(MATH)

clean:
	rm $(PROG_NAME)