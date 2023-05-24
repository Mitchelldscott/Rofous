#include "lsm6dsox.h"

LSM6DSOX::LSM6DSOX() {
	/*
		Set up the jawns and the jimmys, config is hardcoded atm
		but that can change to use a more global definition.
	*/	

	sensor_index = 0;
	// LSM6DSOX Setup
	lsm6dsox.begin_I2C();
	lsm6dsox.setAccelRange(IMU_A_RANGE);
	lsm6dsox.setGyroRange(IMU_G_RANGE);
	lsm6dsox.setAccelDataRate(IMU_A_DATA_RATE);
	lsm6dsox.setGyroDataRate(IMU_G_DATA_RATE);

	/*!
		@brief Sets the INT1 and INT2 pin activation mode
		@param active_low true to set the pins  as active high, false to set the
		mode to active low
		@param open_drain true to set the pin mode as open-drain, false to set the
		mode to push-pull
	*/
	lsm6dsox.configIntOutputs(false, true);

	/*!
		@brief Enables and disables the data ready interrupt on INT 1.
		@param drdy_temp true to output the data ready temperature interrupt
		@param drdy_g true to output the data ready gyro interrupt
		@param drdy_xl true to output the data ready accelerometer interrupt
		@param step_detect true to output the step detection interrupt (default off)
		@param wakeup true to output the wake up interrupt (default off)
	*/
	lsm6dsox.configInt1(false, false, false, false, false);

	/*!
		@brief Enables and disables the data ready interrupt on INT 2.
		@param drdy_temp true to output the data ready temperature interrupt
		@param drdy_g true to output the data ready gyro interrupt
		@param drdy_xl true to output the data ready accelerometer interrupt
	*/
	lsm6dsox.configInt2(false, false, false);

	// LIS3MDL Setup
	lis3mdl.begin_I2C();
	lis3mdl.setRange(IMU_M_RANGE);
	lis3mdl.setDataRate(IMU_M_DATA_RATE);
	lis3mdl.setPerformanceMode(IMU_M_PERFORMANCE);
	lis3mdl.setOperationMode(IMU_M_OP_MODE);
	
	lis3mdl.configInterrupt(false, false, false, // enable z axis
											true, // polarity
											false, // don't latch
											true); // enabled!
}

void LSM6DSOX::read_lsm6dsox_accel(){
	/*
		Get the jawns from the jimmys
	*/
	lsm6dsox.readAcceleration(data[0], data[1], data[2]);

	// Serial.print("LSM6DSOX accel read time: "); Serial.println(micros() - read_start);
}

void LSM6DSOX::read_lsm6dsox_gyro(){
	/*
		Get the jawns from the jimmys
	*/
	lsm6dsox.readGyroscope(data[3], data[4], data[5]);
	// Serial.print("LSM6DSOX gyro read time: "); Serial.println(micros() - read_start);
}

void LSM6DSOX::read_lis3mdl(){
	/*
		Get the jawns from the jimmys
	*/
	lis3mdl.readMagneticField(data[6], data[7], data[8]);

	// Serial.print("LIS3MDL mag read time: "); Serial.println(micros() - read_start);
}

void LSM6DSOX::setup() {
	Serial.println("LSM6DSOX setup");
		
}

void LSM6DSOX::reset() {
	Serial.println("LSM6DSOX reset");
}

void LSM6DSOX::clear() {
	Serial.println("LSM6DSOX clear");
}

float* LSM6DSOX::state() {
	float state = float(sensor_index);
	return &state;
}

float* LSM6DSOX::run(float*) {
	switch (sensor_index) {
		case 0:
			read_lsm6dsox_accel();
			sensor_index ++;

		case 1:
			read_lsm6dsox_gyro();
			sensor_index ++;

		case 2:
			read_lis3mdl();
			sensor_index = 0;
	}

	return data;
}

void LSM6DSOX::print() {
	Serial.println("LSM6DSOX");
	Serial.printf("\tsensor_index: %i\n", sensor_index);
	Serial.printf("\taccel: [%f, %f, %f]\n", data[0], data[1], data[2]);
	Serial.printf("\tgyro: [%f, %f, %f]\n", data[3], data[4], data[5]);
	Serial.printf("\tmag: [%f, %f, %f]\n", data[6], data[7], data[8]);
}