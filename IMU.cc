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
  // Init variables:
  angles = new vector(3);
  corrected_accel = new vector(3);
  angular_velocity = new vector(3);
  
  // Init all the sensors
  accel = new BMA020_ACCEL();
  if (!accel.init(I2CBUS_SENSORS)) {
    fprintf(stderr, "FAILED to init the accelerometer (BMA020) on i2c bus %d\nTerminating...", I2CBUS_SENSORS);
    exit(1);
  }
  // Reset all the states
  this->reset();
}

IMU::~IMU() {
  // Free the sensors
  delete accel;
}

void IMU::reset() {
  for (int i = 0; i<3; i++) {
    angles->set(i,0);
    corrected_accel->set(i,0);
    angular_velocity->set(i,0);
  }
  height = 0;
  
  return;
}