/*
 * 5HC99 Quadcopter project
 * SRF02 ultrasound range finder driver (i2c)
 * Group 1, Marco Cox, m.g.h.cox@student.tue.nl
 */

#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/i2c-dev-user.h>
#include <time.h>
#include "SRF02.h"

/********************
 * PUBLIC FUNCTIONS
 ********************/
 
SRF02_US::SRF02_US() {
	handle = 0;
	lastRange = 0;
	measurementBusy = 0;
	endWait = clock();
  smoothing = SRF02_DEFAULT_SMOOTHING;
}

SRF02_US::~SRF02_US() {
	if (this->handle > 0) close(this->handle);
}

int SRF02_US::init(int i2c_bus) {
	// Try to open the i2c bus and connect to the sensor
	// To verify communication, a test register (0x01) is read, which should return SRF02_VERIFICATION
	
	// Return: 1 if successful, 0 if not
	
	char filename[20];
	
	if (this->handle) return 0; // Already init
	snprintf(filename, 20, "/dev/i2c/%d", i2c_bus);
	filename[19] = '\0';

	// Find the correct file and open it
	this->handle = open(filename, O_RDWR);

	if (this->handle < 0 && (errno == ENOENT || errno == ENOTDIR)) {
		sprintf(filename, "/dev/i2c-%d", i2c_bus);
		this->handle = open(filename, O_RDWR);
	}

	if (this->handle < 0 && !SRF02_QUIET) {
		if (errno == ENOENT) {
			fprintf(stderr, "Error SRF02: Could not open handle "
				"`/dev/i2c-%d' or `/dev/i2c/%d': %s\n",
				i2c_bus, i2c_bus, strerror(ENOENT));
		} else {
			fprintf(stderr, "Error SRF02: Could not open handle "
				"`%s': %s\n", filename, strerror(errno));
			if (errno == EACCES)
				fprintf(stderr, "Run as root?\n");
		}
	}
	if (this->handle < 0) return 0;
	
	// Set the address of the slave
	/* With force, let the user read from/write to the registers
	   even when a driver is also running */
	if (ioctl(this->handle, SRF02_FORCE ? I2C_SLAVE_FORCE : I2C_SLAVE, SRF02_ADDRESS) < 0) {
		if (!SRF02_QUIET) {
			fprintf(stderr,
				"Error SRF02: Could not set address to 0x%02x: %s\n",
				SRF02_ADDRESS, strerror(errno));
		}
		return 0;
	}

	// To verify communication, a test register (0x01) is read, which should return SRF02_VERIFICATION
	int res = this->readByte(0x01);
	if (res < 0) {
		if (!SRF02_QUIET) {
			fprintf(stderr, "Error SRF02: Reading the test register (address 0x01) failed\n");
		}
		return 0;
	}
	if (res != SRF02_VERIFICATION) {
		if (!SRF02_QUIET) {
			fprintf(stderr, 
				"Error SRF02: Test register value does not match. Read 0x%02x, should be 0x%02x.\n", 
				res, SRF02_VERIFICATION);
		}
		return 0;
	}
	
	return 1;
}

unsigned int SRF02_US::getRange() {
	if (!this->measurementBusy && (clock()>this->endWait)) {
		this->startMeasurement();
	}
	if (this->measurementBusy && (clock()>this->endWait)) {
		this->saveMeasurement();
	}
	return (this->lastRange>0) ? this->lastRange : 0;
}

/********************
 * PRIVATE FUNCTIONS
 ********************/

void SRF02_US::startMeasurement() {
	if (!(this->handle>0)) {
		if (!SRF02_QUIET) {
			fprintf(stderr, "Error SRF02: Cannot start measurement when not connected.\n");
		}
		return;
	}
	this->measurementBusy = 1;
	this->writeByte(SRF02_ADDR_CMD, SRF02_CMD_RANGE);
	// Now make sure we wait 70ms before asking for the result
	this->endWait = clock() + SRF02_DELAY * CLOCKS_PER_SEC;
	return;
}

void SRF02_US::saveMeasurement() {
	if (!(this->handle>0)) {
		if (!SRF02_QUIET) {
			fprintf(stderr, "Error SRF02: Cannot save measurement when not connected.\n");
		}
		return;
	}
	if (!(clock()>this->endWait)) return;	// Wait some more time!
	
	int range = i2c_smbus_read_word_data(this->handle, SRF02_ADDR_RANGE);
	if (range<0 || range>SRF02_RANGE_LIMIT) {
		if (!SRF02_QUIET) {
			fprintf(stderr, "Error SRF02: Unrealistic range measurement: %d\n", range);
		}	
		range = -1;
	}
	this->lastRange = smoothing * this->lastRange + (1-smoothing) * range;
	this->measurementBusy = 0;
	this->endWait = clock() + SRF02_DELAY * CLOCKS_PER_SEC;	// Wait for echo to fade away
	return;
}

int SRF02_US::readByte(int address) {
	if (!(this->handle>0)) return 0;	// Not connected to sensor
	int res = i2c_smbus_read_word_data(this->handle, address);
	if (res<0) {
		if (!SRF02_QUIET) {
			fprintf(stderr, 
				"Error SRF02: Could not read some data register (0x%02x) on the sensor.\n",
				address);
		}
		return -1;
	} else {
		return res;
	}
}

int SRF02_US::writeByte(int address, unsigned char data) {
	if (!(this->handle>0)) return 0;	// Not connected to sensor
	int res = i2c_smbus_write_word_data(this->handle, address, data);
	if (res<0) {
		if (!SRF02_QUIET) {
			fprintf(stderr, 
				"Error SRF02: Could not write some data register (0x%02x) on the sensor.\n",
				address);
		}
		return -1;
	} else {
		return 1;
	}
}
