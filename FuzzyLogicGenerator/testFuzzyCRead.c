#include <stdlib.h>
#include <stdio.h>
#include "fMatrices_servoFuzzyLogic.h"

int main(int argc, char **argv) {

	for (int i = 0; i < 21; i++) {
		for (int j = 0; j < 21; j++) {
			for (int k = 0; k < 21; k++) {
				printf("%f, %f, %f, %f, %f\n", FUZZYLOOKUP[i][j][k][0], FUZZYLOOKUP[i][j][k][1], FUZZYLOOKUP[i][j][k][2], FUZZYLOOKUP[i][j][k][3], FUZZYLOOKUP[i][j][k][4]);
			}
		}
	}
	return 0;
}