// Just a Quick&Dirty tool to calibrate the sensors
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

BMA020_ACCEL* accel;
Vector3d A_opt[6];
Vector3d A_raw[6];

void collectData();
void waitKey();

int main(int argc, char *argv[]) {
	accel = new BMA020_ACCEL;
	accel->use_calibration = 0;	// Disable calibration, we want raw measurements!

	A_opt[0] = new Vector3d( 0 , 0 , 1 );
	A_opt[1] = new Vector3d( 0 , 0 ,-1 );
	A_opt[2] = new Vector3d( 0 , 1 , 0 );
	A_opt[3] = new Vector3d( 0 ,-1 , 0 );
	A_opt[4] = new Vector3d( 1 , 0 , 0 );
	A_opt[5] = new Vector3d(-1 , 0 , 0 );
	
	for (int i=0;i<6;i++) {
		A_raw[i] = new Vector3d(0,0,0);
	}
	
	if (accel->init(3)) {
		printf("Init of accelerometer successful!!\n\n");
	} else {
		printf("Init of accelerometer failed!!\n\n");
		delete accel;
		return -1;
	}
	
	// Fill A_raw with data from different orientations:
	collectData();
	
	printf("THANKS Buddy! Here's the raw data: \n\n");
	
	for (int i=0; i<6; i++) {
		printf("(%.2f\t,%.2f\t,%.2f\t)\n", A_raw[i].x, A_raw[i].y, A_raw[i].z);
	}
	
	printf("\nDONE! The values are stored in calibration_*.txt files\n");

	delete accel;
	return 0;
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
	Vector3d accelMeasure;
	float scale = 0.005;
	
	// Roll=0deg, Pitch=0deg. "Flat. Up direction: [0,0,1]"
	printf("Put the quad in the position roll=0deg, pitch=0deg.\nUp direction: (0,0,1). Press key when ready...\n");
	waitKey();
	printf("  Measuring, keep still!!\n");
	for (int n=0;n<200;n++) {
		if (!accel->getMeasurement(&accelMeasure)) exit(1);
		A_raw[0].x += accelMeasure.x;
		A_raw[0].y += accelMeasure.y;
		A_raw[0].z += accelMeasure.z;
	}
	A_raw[0] = A_raw[0]*scale;
	
	// Roll=180deg, Pitch=0deg. "Upside down Up direction: [0,0,-1]"
	printf("Put the quad in the position roll=180deg, pitch=0deg.\nUp direction: (0,0,-1). Press key when ready...\n");
	waitKey();
	printf("  Measuring, keep still!!\n");
	for (int n=0;n<200;n++) {
		if (!accel->getMeasurement(&accelMeasure)) exit(1);
		A_raw[1].x += accelMeasure.x;
		A_raw[1].y += accelMeasure.y;
		A_raw[1].z += accelMeasure.z;
	}
	A_raw[1] = A_raw[1]*scale;
	
	// Roll=-90deg, Pitch=0deg. "Up direction: [0,1,0]"
	printf("Put the quad in the position roll=-90deg, pitch=0deg.\nUp direction: (0,1,0). Press key when ready...\n");
	waitKey();
	printf("  Measuring, keep still!!\n");
	for (int n=0;n<200;n++) {
		if (!accel->getMeasurement(&accelMeasure)) exit(1);
		A_raw[2].x += accelMeasure.x;
		A_raw[2].y += accelMeasure.y;
		A_raw[2].z += accelMeasure.z;
	}
	A_raw[2] = A_raw[2]*scale;
	
	// Roll=90deg, Pitch=0deg. "Upside down Up direction: [0,-1,0]"
	printf("Put the quad in the position roll=90deg, pitch=0deg.\nUp direction: (0,-1,0). Press key when ready...\n");
	waitKey();
	printf("  Measuring, keep still!!\n");
	for (int n=0;n<200;n++) {
		if (!accel->getMeasurement(&accelMeasure)) exit(1);
		A_raw[3].x += accelMeasure.x;
		A_raw[3].y += accelMeasure.y;
		A_raw[3].z += accelMeasure.z;
	}
	A_raw[3] = A_raw[3]*scale;
	
	// Roll=0deg, Pitch=90deg. "Upside down Up direction: [1,0,0]"
	printf("Put the quad in the position roll=0deg, pitch=90deg.\nUp direction: (1,0,0). Press key when ready...\n");
	waitKey();
	printf("  Measuring, keep still!!\n");
	for (int n=0;n<200;n++) {
		if (!accel->getMeasurement(&accelMeasure)) exit(1);
		A_raw[4].x += accelMeasure.x;
		A_raw[4].y += accelMeasure.y;
		A_raw[4].z += accelMeasure.z;
	}
	A_raw[4] = A_raw[4]*scale;
	
	// Roll=0deg, Pitch=-90deg. "Upside down Up direction: [-1,0,0]"
	printf("Put the quad in the position roll=0deg, pitch=-90deg.\nUp direction: (-1,0,0). Press key when ready...\n");
	waitKey();
	printf("  Measuring, keep still!!\n");
	for (int n=0;n<200;n++) {
		if (!accel->getMeasurement(&accelMeasure)) exit(1);
		A_raw[5].x += accelMeasure.x;
		A_raw[5].y += accelMeasure.y;
		A_raw[5].z += accelMeasure.z;
	}
	A_raw[5] = A_raw[5]*scale;
	
	return;
}