/*
 * 5HC99 Quadcopter project, group 1.
 * Inertial Measurement Unit
 * This class combines all sensor data and estimates the orientation & height of the device
 */
 
#ifndef _IMU_H
#define _IMU_H

// BUS SETTINGS
#define I2CBUS_SENSORS 3

// FILTER SETTINGS
#define IMU_STDWEIGHT_ACCEL 0.05    // Relative to gyro weight
#define IMU_STDWEIGHT_MAGNETO 0.05  // Relative to gyro weight


#include "BMA020.h"
#include "SRF02.h"
#include "matrix.h"

class IMU {
	public:
		IMU();
		~IMU();
    void reset();      // Reset the IMU, should be steady on the ground
  private:
    // Sensors
    BMA020_ACCEL* accel;
    // Variables
    vector* angles;           // Pitch, roll and yaw
    vector* corrected_accel;  // Acceleration corrected for gravity
    vector* angular_velocity;  // Angular velocity of the quadcopter
    float height;             // Height of the quadcopter, 0 until about 18cm
  
    
  
};

#endif