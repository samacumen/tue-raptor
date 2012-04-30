/*
 * 5HC99 Quadcopter project
 * BMA020 accelerometer driver (i2c)
 * Group 1, Marco Cox, m.g.h.cox@student.tue.nl
 */
 
#ifndef _BMA020_H
#define _BMA020_H

#define BMA020_QUIET 0			// Should we shut up if we screw up?
#define BMA020_FORCE 0			// Force use of i2c bus even if device driver is running?
#define BMA020_ADDRESS 0x38	// Address of the sensor on the bus, hardcoded in chip
#define BMA020_CHIP_ID 0x02	// Used to chech communication during init

#define BMA020_ADDR_X 0x2		// Address of the register containing the LSB of the X value
#define BMA020_ADDR_Y 0x4	
#define BMA020_ADDR_Z 0x6

#define BMA020_DEFAULT_RANGE 2 // Default range of sensor (+/- 2g, 4g or 8g)
#define BMA020_DEFAULT_BANDWIDTH 100 // Bandwidth of low-pass filter [Hz]

#include "matrix.h"

class BMA020_ACCEL {
	public:
		BMA020_ACCEL();
		~BMA020_ACCEL();
    // init(): Open connection, test, set the range to default value. 
    // Always call before doing other things
		int init(int i2c_bus);
    // getMeasurement(): Measure, calculate forces and write to data
		int getMeasurement(vector* measurement);
		// setRange(): Set the range of the sensor to +/- 2g, 4g or 8g. Avoid clipping!
    // Optional, only call if you don't want to use the default setting (BMA020_DEFAULT_RANGE)
		void setRange(unsigned char range);
    // setBandwidth(): Set the bandwidth of a low-pass filter to reduce the noise level
    // Possible values: 25, 50, 100, 190, 375, 750, 1500 [Hz]
    // Optional, only call if you don't want to use the default setting (BMA020_DEFAULT_BANDWIDTH)
    void setBandwidth(int bandwidth);
    // Read the current bandwidth setting [Hz]:
    int getBandwidth();
  
  	// Variables:
  	int use_calibration;
	private:
		int handle;							// Handle to the bus
    int bandwidth;          // Bandwidth for low-pass filter
		float scale;						// Scaling factor for calculating forces. Depends on the range.
		void loadCalibration();	// Fill calibration data from file
		int readByte(int address);
		int writeByte(int address, unsigned char data);
		matrix* calibration_matrix;		// Will be read from calibration_accel.txt, first 9 entries
		vector* calibration_offset;		// Will be read from calibration_accel.txt, last 3 entries
};

#endif