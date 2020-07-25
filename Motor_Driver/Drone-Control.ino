/*
  Arduino Nano IOT 33

  This file is for the development of a drone flight control system.
  The microcontroller reads the acceleration and gyro-scope to generate offsets 
  that adjust motors to approach the desired pose.

  The circuit:
  - RPi 3b+ -> Arduino Nano 33 IoT -> 4x SimonK ESCs
                                  -> Camera Gimbal

  created 17 July, 2020
  by Mitchell Scott

  This code is inspired by arduino library examples and 
  online sources.
*/
#include "Quadcopter.h"
#include <Servo.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>

#define CYCLE_TIME 600
#define motor_fr 3
#define motor_fl 6
#define motor_rl 9
#define motor_rr 12
#define throttle 1000


Quadcopter device = Quadcopter(CYCLE_TIME, motor_fr, motor_fl, motor_rl, motor_rr);

BLEService fcService("1010");
BLEUnsignedCharCharacteristic thetaChar("1300", BLEWrite);
BLEUnsignedCharCharacteristic phiChar("0013", BLEWrite);
BLEUnsignedCharCharacteristic kill_switch("6793", BLEWrite);
BLEFloatCharacteristic headingX("0111", BLERead);
BLEFloatCharacteristic headingY("1011", BLERead);
BLEFloatCharacteristic headingZ("1101", BLERead);

float offsets[6];
float ble_reader = 0;
int theta;
int phi;
bool enable_bit = true;


void zero_imu(float offset[6]){
  
  float x, y, z, roll, pitch, yaw = 0.0;
  
  for(int i = 0; i < 10; i++){
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()){
      IMU.readAcceleration(x, y, z);
      IMU.readGyroscope(roll, pitch, yaw);
    }
    offset[0] += x;
    offset[1] += y;
    offset[2] += z;
    offset[3] += roll;
    offset[4] += pitch;
    offset[5] += yaw;
  }

  offset[0] /= 10;
  offset[1] /= 10;
  offset[2] /= 10;
  offset[3] /= 10;
  offset[4] /= 10;
  offset[5] /= 10;
}


void setup() {
                                                        
  Serial.begin(9600);                                   // Initialize the Serial Connection
  
  if (!IMU.begin()) {                                   // Initialize the IMU device
    Serial.println("Failed to initialize IMU!");        //  - Used for gyro/accel stabilization
    while (1);                                          //  - If fails device will freeze
  }

  zero_imu(offsets);                                    // Zero the IMU readings

  if(BLE.begin()){                                      // Initialize the BLE service
    BLE.setLocalName("Flight Controller");              //  - Provides a manual control
    BLE.setAdvertisedService(fcService);
    fcService.addCharacteristic(thetaChar);
    fcService.addCharacteristic(phiChar);
    fcService.addCharacteristic(kill_switch);
    fcService.addCharacteristic(headingX);
    fcService.addCharacteristic(headingY);
    fcService.addCharacteristic(headingZ);
    BLE.addService(fcService);
    BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
    BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
    thetaChar.setEventHandler(BLEWritten, thetaReceiver);
    phiChar.setEventHandler(BLEWritten, phiReceiver);
    kill_switch.setEventHandler(BLEWritten, killReceiver);
    BLE.advertise();
  }
  else
    Serial.println("~~~BLE Device Failed~~~");
  
  device.motors[0].writeMicroseconds(throttle * enable_bit);                         // Set Thrttle to 0 for all motors
  device.motors[1].writeMicroseconds(throttle * enable_bit);                         
  device.motors[2].writeMicroseconds(throttle * enable_bit);
  device.motors[3].writeMicroseconds(throttle * enable_bit);
  
  delay(2500);                                                                      // Allows motors to initialize
}


void loop() {
  unsigned long start = micros();
  float deltas[6];

  BLE.poll(); 

  while(Serial.available() > 0){
    int state = Serial.read();
    switch(state){
      case 0:
        theta = Serial.read();
        phi = Serial.read();
        enable_bit = Serial.read();
        device.spherical2cartesian(theta, phi, enable_bit);
    }
  }

  if(ble_reader >= 1){
    device.spherical2cartesian(theta, phi, enable_bit);
    ble_reader = 0;
  }
  
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()){
    IMU.readAcceleration(deltas[0], deltas[1], deltas[2]);
    IMU.readGyroscope(deltas[3], deltas[4], deltas[5]);
    deltas[0] -= offsets[0];
    deltas[1] -= offsets[1];
    deltas[2] -= offsets[2];
    deltas[3] -= offsets[3];
    deltas[4] -= offsets[4];
    deltas[5] -= offsets[5];
  }
  
  int adjustments[4] = {0, 0, 0, 0};
  device.heading2adjustments(deltas, adjustments);
  device.adjust_motor_speeds(adjustments, enable_bit);
  headingX.writeValue(device.heading[0]);
  headingY.writeValue(device.heading[1]);
  headingZ.writeValue(device.heading[2]);

  unsigned long t = micros();
  while( t - start < CYCLE_TIME){
    t = micros();  
    //Serial.print("Under Cycled: ");
    //Serial.println(t - start);  
  }
}

void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  //Serial.print("Connected event, central: ");
  //Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  //Serial.print("Disconnected event, central: ");
  //Serial.println(central.address());
}

void thetaReceiver(BLEDevice central, BLECharacteristic characteristic) {
  //Serial.println("Theta Heading Recieved: ");
  theta = characteristic.value()[0];
  ble_reader += .5;
}

void phiReceiver(BLEDevice central, BLECharacteristic characteristic) {
  //Serial.println("Phi Heading Recieved: ");
  phi = characteristic.value()[0]; 
  ble_reader += .5;
}

void killReceiver(BLEDevice central, BLECharacteristic characteristic) {
  if(characteristic.value()[0] == '0'){
    //Serial.print("Kill Switch Activated");
    enable_bit = 0;
  }
  else
    enable_bit = 1;
}
