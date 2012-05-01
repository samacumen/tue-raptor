// Tool to calibrate the sensors of the quadcopter
// Quadcopter axis (right hand): 
//	- x towards front (nose)
//	- y towards left (west) 
//	- z towards sky (up)
// Angles: clockwise rotation around axis

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "BMA020.h"
#include "matrix.h"

#define NUM_MEASUREMENTS 100				// Number of measurements in each position to average
#define NUM_POSITIONS 8							// Number of positions for accel/compass

BMA020_ACCEL* accel;
matrix* A_opt;
matrix* A_raw;
vector* myFavoritePositions[NUM_POSITIONS];	// Pitch, Roll, Yaw

void collectData();
void waitKey();
void init();	// Fill all the matrices & stuff

int main(int argc, char *argv[]) {
	FILE * calibFile;
	printf("Welcome to the calibration tool for the quadcopter\n");
	printf("Active calibration functions: ACCELEROMETER\n");
	printf("Please type 'q' to quit if you lose your patience. Good luck!\n\n");
	
	init();
	
	// Connect to the sensors, disable calibration of the sensor outputs
	accel = new BMA020_ACCEL;
	accel->use_calibration = 0;	// Disable calibration, we want raw measurements!
	if (accel->init(3)) {
		printf("Init of accelerometer successful!!\n\n");
	} else {
		printf("Init of accelerometer failed!!\n\n");
		delete accel;
		return -1;
	}
	
	// Fill A_raw with data from different orientations:
	collectData();
	
	printf("\nTHANKS Buddy! Now going to calculate optimal calibration values...\n\n");
	
	// Calculate accelCalib matrix by finding the least squares solution (pseudo-inverse)
	// accelCalib is a 3x4 matrix
	//  - The first 3x3 part is the calibration matrix
	//  - Te last column is the offset vector
	matrix accelCalib = (*A_opt) * (*A_raw).pseudo_inverse();
	// Write the values to calibrate/accel.txt:
	remove("calibrate/accel.txt");
  calibFile = fopen ("calibrate/accel.txt","w");
  for (int i=0; i<3; i++) {
  	for (int j=0; j<3; j++) {
			fprintf(calibFile, "%.4f\n", accelCalib.data[i][j]);
		}
	}
	for (int i=0; i<3; i++) {
		fprintf(calibFile, "%.4f\n", accelCalib.data[i][3]);
	}
	fclose(calibFile);
	
	// One should clean up his own crap:
	delete accel;
	// Things that are loaded in init():
	for (int i=0; i<NUM_POSITIONS; i++) delete myFavoritePositions[i];
	delete A_opt;
	delete A_raw;
	return 0;
}

void init() {
	// Positions (pitch, roll, yaw) in degrees
	myFavoritePositions[0] = new vector(0,0,0);			// BeagleBone on top
	myFavoritePositions[1] = new vector(90,0,0);
	myFavoritePositions[2] = new vector(180,0,0);		// Zippy on top
	myFavoritePositions[3] = new vector(-90,0,0);
	myFavoritePositions[4] = new vector(0,90,0);
	myFavoritePositions[5] = new vector(0,-90,0);
	myFavoritePositions[6] = new vector(0,0,90);
	myFavoritePositions[7] = new vector(0,0,-90);
	
	
	A_opt = new matrix(3,NUM_POSITIONS);	// The 'should be' values for the accelerometer
	A_opt->data[0][0] = 0; 	A_opt->data[1][0] = 0; 	A_opt->data[2][0] = 1;
	A_opt->data[0][1] = 1; 	A_opt->data[1][1] = 0; 	A_opt->data[2][1] = 0;
	A_opt->data[0][2] = 0; 	A_opt->data[1][2] = 0; 	A_opt->data[2][2] = -1;
	A_opt->data[0][3] = -1; A_opt->data[1][3] = 0; 	A_opt->data[2][3] = 0;
	A_opt->data[0][4] = 0; 	A_opt->data[1][4] = 1; 	A_opt->data[2][4] = 0;
	A_opt->data[0][5] = 0; 	A_opt->data[1][5] = -1; A_opt->data[2][5] = 0;
	A_opt->data[0][6] = 0; 	A_opt->data[1][6] = 0; 	A_opt->data[2][6] = 1;
	A_opt->data[0][7] = 0; 	A_opt->data[1][7] = 0; 	A_opt->data[2][7] = 1;
	
	A_raw = new matrix(4,NUM_POSITIONS);
	for (int i=0; i<NUM_POSITIONS; i++) {
		// Initiate x,y,z to 0 because we sum the outputs to average multiple measurements
		A_raw->data[0][i] = 0;	// x
		A_raw->data[1][i] = 0;	// y
		A_raw->data[2][i] = 0;	// z
		A_raw->data[3][i] = 1;	// For offset calculation
	}
	
	return;
}

void waitKey() {
	char key;
	scanf("%c",&key);
	if (key=='q') {
		exit(1);
	}
	return;
}


void collectData() {
	/*
	 * Collect orientation data for accelerometer + compass
	 */
	float avgScale = 1/NUM_MEASUREMENTS;
	vector* accelMeasure = new vector(3);
	
	for (int i=0; i<NUM_POSITIONS; i++) {
		printf("\nPut the quad in the position roll=%.2f deg, pitch=%.2f deg, yaw=%.2f deg.\nPress key when ready...\n", (*myFavoritePositions[i])[0], (*myFavoritePositions[i])[1], (*myFavoritePositions[i])[2]);
		waitKey();
		printf(">> Measuring, keep still!!\n");
		for (int n=0;n<NUM_MEASUREMENTS;n++) {
			// Accelerometer
			if (!accel->getMeasurement(accelMeasure)) exit(1);
			A_raw->data[0][i] += (*accelMeasure)[0];
			A_raw->data[1][i] += (*accelMeasure)[1];
			A_raw->data[2][i] += (*accelMeasure)[2];
		}
		// Multiply by avgScale to get the true average instead of the sum of samples
		A_raw->data[0][i] = A_raw->data[0][i]*avgScale;
		A_raw->data[1][i] = A_raw->data[0][i]*avgScale;
		A_raw->data[2][i] = A_raw->data[0][i]*avgScale;
	}
	delete accelMeasure;
	
	return;
}