/*
 * 5HC99 Quadcopter project, group 1.
 * BMA020 accelerometer driver (i2c)
 */

#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/i2c-dev-user.h>
#include "BMA020.h"
#include "matrix.h"

/********************
 * PUBLIC FUNCTIONS
 ********************/
 
BMA020_ACCEL::BMA020_ACCEL() {
	this->calibration_matrix = new matrix(3,3);
  this->calibration_offset = new vector(3);
	handle = 0;
	scale = 0;
  bandwidth = 0;
  use_calibration = 1;
}

BMA020_ACCEL::~BMA020_ACCEL() {
	if (this->handle > 0) close(this->handle);
}

int BMA020_ACCEL::init(int i2c_bus) {
	// Try to open the i2c bus and connect to the sensor
	// To verify communication, the chip_id is read (register 0x00), which should be 0x02
	
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

	if (this->handle < 0 && !BMA020_QUIET) {
		if (errno == ENOENT) {
			fprintf(stderr, "Error BMA020: Could not open handle "
				"`/dev/i2c-%d' or `/dev/i2c/%d': %s\n",
				i2c_bus, i2c_bus, strerror(ENOENT));
		} else {
			fprintf(stderr, "Error BMA020: Could not open handle "
				"`%s': %s\n", filename, strerror(errno));
			if (errno == EACCES)
				fprintf(stderr, "Run as root?\n");
		}
	}
	if (this->handle < 0) return 0;
	
	// Set the address of the slave
	/* With force, let the user read from/write to the registers
	   even when a driver is also running */
	if (ioctl(this->handle, BMA020_FORCE ? I2C_SLAVE_FORCE : I2C_SLAVE, BMA020_ADDRESS) < 0) {
		if (!BMA020_QUIET) {
			fprintf(stderr,
				"Error BMA020: Could not set address to 0x%02x: %s\n",
				BMA020_ADDRESS, strerror(errno));
		}
		return 0;
	}

	// Read the chip-id, compare it with predefined value (BMA020_CHIP_ID) as a check
	int res = i2c_smbus_read_byte_data(this->handle, 0x00);
	if (res < 0) {
		if (!BMA020_QUIET) {
			fprintf(stderr, "Error BMA020: Reading the chip-id (address 0x00) failed\n");
		}
		return 0;
	}
	if (res != BMA020_CHIP_ID) {
		if (!BMA020_QUIET) {
			fprintf(stderr, 
				"Error BMA020: Chip-id does not match. Read 0x%02x, should be 0x%02x.\n", 
				res, BMA020_CHIP_ID);
		}
		return 0;
	}
	
	this->setRange(BMA020_DEFAULT_RANGE);
  this->setBandwidth(BMA020_DEFAULT_BANDWIDTH);
  this->loadCalibration();
	return 1;
}

void BMA020_ACCEL::setRange(unsigned char range) {
	// Set the range of the sensor
	// Possible values: 2, 4 or 8 (+/- g)
	if (range!=2 && range!=4 && range!=8) return;
	// Create bitmask
	int rangeBin;
	if (range==2) rangeBin = 0x0;
	else if (range==4) rangeBin = 0x1;
	else if (range==8) rangeBin = 0x2;
	
	// Read the current value of the register, set the correct bits and write it back
	int a14 = this->readByte(0x14);
	if (a14<0) return;
	a14 &= 0xE7;
	a14 |= (rangeBin<<3);
	a14 = this->writeByte(0x14, (unsigned char)a14);
	if (a14<0) return;	
	this->scale = ((float)(range*2))/1024;
	return;
}

void BMA020_ACCEL::setBandwidth(int bandwidth) {
  if (bandwidth!=25 && bandwidth!=50 && bandwidth!=100 
      && bandwidth!=190 && bandwidth!=375 && bandwidth!=750 
      && bandwidth!=1500) return;
  int bwBin;
	if (bandwidth==25) bwBin = 0x0;
	else if (bandwidth==50) bwBin = 0x1;
	else if (bandwidth==100) bwBin = 0x2;
  else if (bandwidth==190) bwBin = 0x3;
  else if (bandwidth==375) bwBin = 0x4;
  else if (bandwidth==750) bwBin = 0x5;
  else if (bandwidth==1500) bwBin = 0x6;
  
	// Read the current value of the register, set the correct bits and write it back
	int a14 = this->readByte(0x14);
	if (a14<0) return;
	a14 &= 0xF8;
	a14 |= bwBin;
	a14 = this->writeByte(0x14, (unsigned char)a14);
	if (a14<0) return;	
	this->bandwidth = bandwidth;
  return;
}

int BMA020_ACCEL::getBandwidth() {
  return this->bandwidth;
}

int BMA020_ACCEL::getMeasurement(vector* measurement) {
	if (!(this->handle>0)) return 0;	// Not connected to sensor
	if (!(this->scale>0)) return 0;		// No valid scale
	
	// Read the data in parts of 2 bytes
	int x = i2c_smbus_read_word_data(this->handle, BMA020_ADDR_X);
	int y = i2c_smbus_read_word_data(this->handle, BMA020_ADDR_Y);
	int z = i2c_smbus_read_word_data(this->handle, BMA020_ADDR_Z);
	if (x<0 || y<0 || z<0) {
		if (!BMA020_QUIET) {
			fprintf(stderr, "Error BMA020: Could not read some data register on the sensor.\n");
		}
		return 0;
	}
	
	x = x>>6;	// Those are now values from 0...1023 (10 bit two's complement)
	y = y>>6;
	z = z>>6;
	// Convert values to signed:
	if (x&0x200) x = -1024 + x; // Those are now values from -512...511
	if (y&0x200) y = -1024 + y; // Those are now values from -512...511
	if (z&0x200) z = -1024 + z; // Those are now values from -512...511
	
	vector data = new vector(x*this->scale, y*this->scale, z*this->scale);
	if (this->use_calibration) {
		*measurement = (*this->calibration_matrix)*data + (*this->calibration_offset);
	} else {
		*measurement = data;	
	}
	return 1;
}

/********************
 * PRIVATE FUNCTIONS
 ********************/

void BMA020_ACCEL::loadCalibration() {
	FILE * cFile;
  cFile = fopen ("calibrate/accel.txt","r");
  // Calibration matrix
  for (int i=0; i<3; i++) {
  	for (int j=0; j<3; j++) {
  		fscanf (cFile, "%f", &(this->calibration_matrix->data[i][j]));
  	}
  }
  // Calibration offset
  float temp;
  fscanf (cFile, "%f", &temp);
  this->calibration_offset->set(0,temp);
  fscanf (cFile, "%f", &temp);
  this->calibration_offset->set(0,temp);
  fscanf (cFile, "%f", &temp);
	this->calibration_offset->set(0,temp);
	
  fclose (cFile);
  
}

int BMA020_ACCEL::readByte(int address) {
	if (!(this->handle>0)) return 0;	// Not connected to sensor
	int res = i2c_smbus_read_byte_data(this->handle, address);
	if (res<0) {
		if (!BMA020_QUIET) {
			fprintf(stderr, 
				"Error BMA020: Could not read some data register (0x%02x) on the sensor.\n",
				address);
		}
		return -1;
	} else {
		return res;
	}
}

int BMA020_ACCEL::writeByte(int address, unsigned char data) {
	if (!(this->handle>0)) return 0;	// Not connected to sensor
	int res = i2c_smbus_write_word_data(this->handle, address, data);
	if (res<0) {
		if (!BMA020_QUIET) {
			fprintf(stderr, 
				"Error BMA020: Could not write some data register (0x%02x) on the sensor.\n",
				address);
		}
		return -1;
	} else {
		return 1;
	}
}
