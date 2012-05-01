/*
 * 5HC99 Quadcopter project, group 1.
 * Inertial Measurement Unit
 * This class combines all sensor data and estimates the orientation & height of the device
 */
 
#ifndef _IMU_H
#define _IMU_H

#include "BMA020.h"
#include "SRF02.h"
#include "matrix.h"

class IMU {
	public:
		IMU();
		~IMU();
    // init(): Open connection, test, set the range to default value. 
    // Always call before doing other things
		int init(int i2c_bus);
  private:

};

#endif