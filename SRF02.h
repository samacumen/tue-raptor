/*
 * 5HC99 Quadcopter project, group 1.
 * SRF02 ultrasound range finder driver (i2c)
 */
 
#ifndef _SRF02_H
#define _SRF02_H

#define SRF02_QUIET 0				// Should we shut up if we screw up?
#define SRF02_FORCE 0				// Force use of i2c bus even if device driver is running?
#define SRF02_ADDRESS 0xE0	// Address of the sensor on the bus
#define SRF02_VERIFICATION 0x80	// Read value of register 0x01, to test communication
#define SRF02_DELAY 0.07		// Minimum delay in seconds between communications with sensor
#define SRF02_DEFAULT_SMOOTHING 0.3		// New_range = smoothing * old_range + (1-smoothing) * current_range
#define SRF02_ADDR_RANGE 0x2		// Address of the register containing the LSB of the X value
#define SRF02_ADDR_CMD	 0x0		// Address to write command to	

#define SRF02_CMD_RANGE 0x51 		// Command for doing ranging in [cm]
#define SRF02_RANGE_LIMIT 1000 	// Values above this one are not realistic (1000 = 1000cm = 10m)

#include <time.h>

class SRF02_US {
	public:
		SRF02_US();
		~SRF02_US();
    // init(): Open connection, test, set the range to default value. 
    // Always call before doing other things
		int init(int i2c_bus);
    /*
		getRange(): Returns the most recent range in cm.
    PLEASE NOTE: This is the raw range, so not corrected for pitch & roll, which effect the measured range.
		Since the sensor is slow (measuring takes 65ms, another 65ms needed for echo to fade)
		we will return the previous value, and check if we can start a new measurement.
		If a measurement has been initiated more than 70ms ago, the result is loaded from the
		sensor and returned. In this way, the function call to getRange() is fast and measurements
		are done in the background. Drawback is a small lag in the measurements. 
    */
    unsigned int getRange();
    float smoothing;
  
	private:
		int handle;								// Handle to the bus
		int lastRange;						// -1 if no value is present
		int measurementBusy;			// Measurement currently in progress?
		clock_t endWait;					// Used to save when we are free to use the sensor
		void startMeasurement();	// Initiate new measurement
		void saveMeasurement();		// Load value from sensor into lastRange.
		int readByte(int address);
		int writeByte(int address, unsigned char data);
};

#endif