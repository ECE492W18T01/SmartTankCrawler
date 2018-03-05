#include <stdlib.h>
#include <stdio.h>
#include "sliplessSteeringRatios_45R_17W_30L.h"

int main(int argc, char **argv) {

	for (int i = 0; i < 46; i++) {
		printf("%d, %d, %d\n", SLIPRATIOS[i][0], SLIPRATIOS[i][1], SLIPRATIOS[i][2]);
	}
	return 0;
}