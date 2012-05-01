/*
 * 5HC99 Quadcopter project, group 1.
 * Inertial Measurement Unit
 * This class combines all sensor data and estimates the orientation & height of the device
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "IMU.h"
#include "BMA020.h"
#include "SRF02.h"
#include "matrix.h"

/********************
 * PUBLIC FUNCTIONS
 ********************/
 
IMU::IMU() {
  // TODO
}

IMU::~IMU() {
  // TODO
}

int IMU::init(int i2c_bus) {
  // TODO
  return 1;
}
