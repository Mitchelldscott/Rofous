#include "OneLiner.h"
#include <Arduino_LSM6DS3.h>

#define NAME "Rofous_FW"
#define CYCLE_TIME 200l

float vel[6] = {0.0, 0.0, 0.0};
float pose[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float sensor[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

unsigned long start;
unsigned long duration;
unsigned long birthTime;

OneLiner OL;

void serialEvent() {
	/*
		serialEvent is a serial callback.
		but it doesn't ucking wrk on IOT nano33!!
				(workaround: call in loop)
	*/
	asset_cmd = "";
	while (Serial.available()) {
		char inChar = (char)Serial.read();
		if (inChar == '\n')
			break;

		asset_cmd += inChar;
	}
	
	if (asset_cmd == "KILL")
		OL.writeMsg("SUDO");
	if (asset_cmd == "WHOAMI")
		OL.writeMsg(NAME, NAME);
}

void readIMU(float* pose){
	if (IMU.accelerationAvailable())
		IMU.readAcceleration(pose[0], pose[1], pose[2]);

	if (IMU.gyroscopeAvailable())
		IMU.readGyroscope(pose[3], pose[4], pose[5]);
}

void publishIMU(float* pose)
{
	OL.writeMsg("IMU", pose, 6);
}

void updateOdometry(float* sensor, float* pose){
	for (int i=0; i < 3; i++){
		vel[i] += sensor[i] * (CYCLE_TIME * 0.0001);
		pose[i] += vel[i] * (CYCLE_TIME * 0.0001);
		pose[i + 3] += sensor[i + 3] * (CYCLE_TIME * 0.0001);
	}
}

void setup() {
	birthTime = millis();

	Serial.begin(9600);
	while (!Serial);

	if (!IMU.begin()) {
		OL.writeMsg("Error", "IMU not initialized");
		while (1);
	}

	OL.writeMsg(NAME, NAME);
}

void loop() {
	start = millis();
	readIMU(sensor);									// red IMU sensor registers
	updateOdometry(sensor, pose);			// update pose based on measurements
	publishIMU(pose);									// write the pose over Serial

	serialEvent();										// handle any serial events

	
	////////		Normalize Cycle time		////////
	duration = millis() - start;
	if (duration < CYCLE_TIME)
		delay(CYCLE_TIME - duration);

}
