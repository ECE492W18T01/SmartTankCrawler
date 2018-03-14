#include <stdlib.h>
#include <stdio.h>
#include "motorDriveR.h"

int main(int argc, char **argv) {

	for (int i = 0; i < 11; i++) {
		printf("%f, %f, %f\n", DRIVERATIOS[i][0], DRIVERATIOS[i][1], DRIVERATIOS[i][2]);
	}
	return 0;
}