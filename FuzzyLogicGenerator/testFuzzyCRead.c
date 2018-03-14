#include <stdlib.h>
#include <stdio.h>
#include "fMatrices_servoFuzzyThree.h"

/* 
 * ECE 492 
 * Wi18
 * Keith Mills
 * Just a simple file for confirming that the results of 
 * generateDecisionMatrices.m can be read properly by C and compile with
 * a program.
 * 
 * Compile using
 * gcc testFuzzyCRead.c fMatrices_servoFuzzyThree.h fMatrices_servoFuzzyThree.c -o fr -std=c99 -Wall
 * And run using ./fuzzy
 * All it does is read the matrices.
*/
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