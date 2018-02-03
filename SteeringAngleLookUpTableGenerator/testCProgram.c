#include <stdlib.h>
#include <stdio.h>
#include "sliplessSteeringRatios_33R_3W_10L.h"

int main(int argc, char **argv) {

	for (int i = 0; i < 33; i++) {
		printf("%d, %d, %d\n", SLIPRATIOS[i][0], SLIPRATIOS[i][1], SLIPRATIOS[i][2]);
	}
	return 0;
}