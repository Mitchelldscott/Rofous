// #include "sensors/lsm6dsox.h"
// #include "robot_comms/hid_report.h"
// #include "algorithms/complimentary_filter.h"

// #ifndef BUFF_DEVICE_MANAGER_H
// #define BUFF_DEVICE_MANAGER_H

// #define NUM_PROPS 4
// #define ATTITUDE_DIM 3

// /*
// 	An organizer for all the pipelines with the firmware.
// 	A large portion of this is HID handling (lots of switches)
// 	Drivers should provide a good api for this object to use them.

// 	The idea is that by organizing drivers here our main is 
// 	dummy simple and easy to adjust the scheduling.
// */

// // class Data_Store {
// // 	private:
// // 		// Vector_Set sensor_buffer_cache_head; // linked list of vectors
// // 		// Vector_Set motor_buffer_cache_head; // linked list of vectors
// // 		// Vector_Set proc_output_cache_head; // linked list of vectors
// // 		Cerebellum motor_head;
// // 		Sensory_Cortex sensor_head;
// // 		Motor_Cortex process_head;  // graph of processes

// // 	public:
// // 		Data_Store();
// // 		new_motor();
// // 		new_sensor();
// // 		new_process();
// // 		read_sensors();
// // 		write_motors();
// // 		step_process();
// // };

// struct Device_Manager {	
// 	Device_Manager();

// 	// handlers for HID reports
// 	void initializer_report_handle();
// 	void feedback_request_handle();
// 	void control_input_handle();
// 	void sensor_request_handle();

// 	// More HID report handlers
// 	void report_switch();
// 	void hid_input_switch(uint32_t);

// 	// Non-HID related pipelines
// 	// void push_can();
// 	void read_sensors();			// add functionality to read the Rev encoders (in addition to another sensor)
// 	void step_controllers(float);

// 	// Data Store
// 	// this is the cylinder in the flow chart 
// 	// drivers will put their data here
// 	Hid_Report input_report;	// message being read on teensy from computer (request or command)
// 	Hid_Report output_report;	// data being sent from teensy to computer

// 	// ADD new IMU here, also make sure the 
// 	// senor pipeline and dev manager constructors knows about it!!
// 	LSM6DSOX imu;

// 	ComplimentaryFilter attitude_filter;

// 	// struct containing all the goods
// 	Data_Store ds;

// 	// track the time since a connection was made
// 	float lifetime;

// 	// Logic for state machines (literally which case to run)
// 	// see implementation for more details
// 	int sensor_switch;
// };

// #endif