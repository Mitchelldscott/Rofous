#include "buff_cpp/blink.h"
#include "buff_cpp/timing.h"
#include "buff_cpp/device_manager.h"

// Uses builtin LED to show when HID is connected
Device_Manager::Device_Manager(){
	setup_blink();
	timer_set(2);
	timer_set(3);
}

/*
	There are a set of initializer reports required by drivers 
	to configure the devices. This is the case handler
	for each of those.

	TODO:
		make this more generic so there is not
	a new case for each driver.

	Author: Mitchell Scott
*/
void Device_Manager::initializer_report_handle() {
	output_report.put(1, input_report.get(1));
	output_report.put(2, input_report.get(2));

	switch (input_report.get(1))	{
		case 0:
			if (input_report.get(2) == 0) {
				attitude_filter.set_gain(input_report.get_float(3));
			}
			break;

		default:
			break;
	}
}

/*
	The device manager will continously read from can devices.
	This data is put in the DM Store. The HIDLayer
	will request these samples. This is the handler for
	that.

	TODO:
		

	Author: Mitchell Scott
*/
void Device_Manager::feedback_request_handle() {
	int request_mode = input_report.get(1);
	output_report.put(1, request_mode);

	switch (request_mode) {
		case 1:
			output_report.put(2, 0);
			for (int i = 0; i < ATTITUDE_SIZE; i++){
				output_report.put_float((4 * i) + 3, ds.attitude[i]);
			}
			break;

		default:
			break;
	}
}

/*
	The device manager will continously update control outputs. 
	The HIDLayer will request reports of this. This is the handler for
	that.

	TODO:
		Add more general data on the controller
		i.e. dumb the control mode, the input and
		timestamps. Currently this only share motor controller
		info. Talk with Mitchell about designing the reports.

	Author: Mitchell Scott
*/
void Device_Manager::control_input_handle() {
	
}

/*
	The device manager will continously read from each of
	the sensors. This data is put in the DM Store. The HIDLayer
	will request each of these samples. This is the handler for
	that.

	TODO:
		Add other IMU

	Author: Mitchell Scott
*/
void Device_Manager::sensor_request_handle() {
	// use input_report.data[1] as the sensor to read
	// imu = 0 (36 bytes = 9 floats), dr16 = 1 (28 bytes = 7 floats)
	int sensor = input_report.get(1);
	output_report.put(1, sensor);

	switch (sensor) {
		case 0:
			for (int i = 0; i < LSM6DSOX_DOF; i++){
				output_report.put_float((4 * i) + 2, ds.imu_measurement[i]);
			}
			break;

		default:
			break;
	}
}

/*
	Input reports will have an ID indicating
	how to handle the report. Do this here

	TODO:
		

	Author: Mitchell Scott
*/
void Device_Manager::report_switch() {
	// reply to the report with the same id
	output_report.put(0, input_report.get(0));

	switch (input_report.get(0)) {
		case 255:
			// configuration / initializers
			initializer_report_handle();
			lifetime = 0;
			break;

		case 1:
			// motor feedback data request
			feedback_request_handle();
			// Serial.println("Motor feedback requested");
			break;

		case 2:
			// controls reports
			control_input_handle();
			break;

		case 3:
			// sensor data request
			sensor_request_handle();
			// Serial.println("Sensor data requested");
			break;

		default:
			break;
	}
}

/*
	Check for an HID packet

	TODO:
		

	Author: Mitchell Scott
*/
void Device_Manager::hid_input_switch(uint32_t cycle_time_us){
	/*
		Check if there is an HID input packet, 
		if there is one check the packet request
		and build the packet.
		@param:
		  None
		@return:
		  If a packet was read or not
	*/
	// assure good timing of hid packets
	// if (timer_info_us(3) > cycle_time_us) {
	// 	Serial.println("HID loop overcycled");
	// }
	// timer_wait_us(3, cycle_time_us);
	lifetime += timer_info_us(3);
	timer_set(3);

	switch (input_report.read()) {
		case 64:
			blink();										// only blink when connected to hid
			report_switch();
			output_report.put_float(60, lifetime);
			break;
		
		default:
			break;
	}

	output_report.write();
	output_report.clear();
}

/*
	Read one of the I2c sensors and any other quick
	to read devices.


	TODO:
		- Add high range IMU driver to state machine
		- Add rev encoder to be read each loop (if fast enough)

	Author: Mitchell Scott
*/
void Device_Manager::read_sensors() {
	switch (sensor_switch) {
		case 0:
			imu.read_lsm6dsox_accel(ds.imu_measurement);
			sensor_switch += 1;
			break;

		case 1:
			imu.read_lsm6dsox_gyro(&ds.imu_measurement[3]);
			sensor_switch += 1;
			break;

		case 2:
			imu.read_lis3mdl(&ds.imu_measurement[6]);
			sensor_switch = 0;
			break;

		default:
			sensor_switch = 0;
			break;
	}
}

/*
	Based on the control switch and the various
	input/feedback in the DM store, compute the
	motor output values = [-1:1].

	controller_switch tells the controller where input
	is coming from. This will need to be relayed to HID.
	Do this from the controller report handler and HIDLayer


	TODO:
		Add different controllers,
			- Gravity compensated
			- Power limited

		These two can inherit or replace
		the feedback controller object. The first two
		feedback values should remain unchanged (position, velocity)
		The third will be dependant on the controller type (maybe handled by each controller)
			power = i * V = v * F (current * Voltage/velocity * torque/rotational velocity * force)
			grav compensation = mglsin(theta), assume params have been identified
			and come from an initializer report.

		We can disscuss removing the velocity reference, this will make the velocity feedack
		a very strong damper i.e. Kd * (Rv - v) -> Kd * v (any non zero v will cause an opposing control force)
		(currently is used to help track velocity, should always have small gain ~< 1 due to noise)

	Author: Mitchell Scott
*/
void Device_Manager::step_controllers(float dt) {
	attitude_filter.filter(ds.imu_measurement, &ds.imu_measurement[3], &ds.imu_measurement[6], dt, ds.attitude);
}
