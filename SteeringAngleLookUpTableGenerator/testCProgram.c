#include <stdlib.h>
#include <stdio.h>
#include "sliplessSteeringRatios.h"

int main(int argc, char **argv) {

	for (int i = 0; i < 11; i++) {
		printf("%d, %d, %d\n", SLIPRATIOS[i][0], SLIPRATIOS[i][1], SLIPRATIOS[i][2]);
	}
	return 0;
}