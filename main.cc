// This is just a test file for the BMA020_ACCEL class

#include <stdio.h>
#include "BMA020.h"
#include "matrix.h"

int main(int argc, char *argv[]) {
	BMA020_ACCEL* mySensor = new BMA020_ACCEL;
	
	if (mySensor->init(3)) {
		printf("Init of sensor successful!!\n\n");
	} else {
		printf("Init of sensor failed!!\n\n");
		delete mySensor;
		return -1;
	}
	
	vector myMeasurement = new vector(3);
	char key;
	printf("Starting measurements. Press any key to continue, press q to quit\n\n");
		
	while (scanf("%c",&key)) {
		if (key=='q') break;
		if (mySensor->getMeasurement(&myMeasurement)) {
			printf("Measurement (x,y,z) = (%.2f,%.2f,%.2f)\n",
				myMeasurement[0],
				myMeasurement[1],
				myMeasurement[2]);
		} else {
			printf("Measurement failed!!\n");
		}
	}
	
	delete mySensor;
	
	return 0;
}