#include "OneLiner.h"
#include <Arduino_LSM6DS3.h>

#define WHOAMI "Rofous_FW"
#define CYCLE_TIME 200ul

float vel[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float pose[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float sensor[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

unsigned long start;
unsigned long duration;
unsigned long birthTime;

char writeBack = '0';

OneLiner OL;
OneLinerMessage nameCard = OL.init("whoami", 0);

bool isConnected = false;

void serialEvent() {
	/*
		serialEvent is a serial callback.
		but it doesn't ucking work on IOT nano33!!
				(workaround: call in loop)
	*/
	while(Serial.available())
		writeBack = Serial.read();
}

void readIMU(float* sensor){
	if (IMU.accelerationAvailable())
		IMU.readAcceleration(sensor[0], sensor[1], sensor[2]);

	if (IMU.gyroscopeAvailable())
		IMU.readGyroscope(sensor[3], sensor[4], sensor[5]);
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
	nameCard.msg = WHOAMI;

	Serial.begin(9600);
	while (!Serial);

	if (!IMU.begin())		
		while (1);
	
	OL.publish(nameCard);
}

void loop() {
	start = millis();
	readIMU(sensor);									// red IMU sensor registers
	updateOdometry(sensor, pose);						// update pose based on measurements

	if(!isConnected)
		OL.publish(nameCard);

	if(Serial.available())
		serialEvent();

	if(writeBack == 13)
		isConnected = true;
	
	////////		Normalize cycle time		////////
	duration = millis() - start;
	if (duration < CYCLE_TIME)
		delay(CYCLE_TIME - duration);
}