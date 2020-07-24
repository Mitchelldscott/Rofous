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

#define CYCLE_TIME 1
#define motor_fr 2
#define motor_fl 3
#define motor_rl 12
#define motor_rr 9
#define throttle 1000

Quadcopter device = Quadcopter(CYCLE_TIME, motor_fr, motor_fl, motor_rl, motor_rr);

BLEService fcService("1010");
BLEUnsignedCharCharacteristic heading("1313", BLEWrite);
BLEUnsignedCharCharacteristic kill_switch("6793", BLEWrite);
BLEFloatCharacteristic poseX("0111", BLERead);
BLEFloatCharacteristic poseY("1011", BLERead);
BLEFloatCharacteristic poseZ("1101", BLERead);

float offsets[6];
int enable_bit = 1;


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
  while (!Serial);                                      //  - Used for debugging and in future will be the main comms for RPi -> Arduino
                                                        //  - If not connected the device will freeze
  if (!IMU.begin()) {                                   // Initialize the IMU device
    Serial.println("Failed to initialize IMU!");        //  - Used for gyro/accel stabilization
    while (1);                                          //  - If fails device will freeze
  }

  zero_imu(offsets);                                    // Zero the IMU readings

  if(BLE.begin()){                                      // Initialize the BLE service
    BLE.setLocalName("Flight Controller");              //  - Provides a manual control
    BLE.setAdvertisedService(fcService);
    fcService.addCharacteristic(heading);
    fcService.addCharacteristic(poseX);
    fcService.addCharacteristic(poseY);
    fcService.addCharacteristic(poseZ);
    fcService.addCharacteristic(kill_switch);
    BLE.addService(fcService);
    BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
    BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
    heading.setEventHandler(BLEWritten, headingReceiver);
    kill_switch.setEventHandler(BLEWritten, killReceiver);
    heading.setValue(0);
    poseX.setValue(0);
    poseY.setValue(0);
    poseZ.setValue(0);
    BLE.advertise();
  }
  else
    Serial.println("~~~BLE Device Failed~~~");
  
  device.motors[0].writeMicroseconds(throttle * enable_bit);                         // Set Thrttle to 0 for all motors
  device.motors[1].writeMicroseconds(throttle * enable_bit);                         
  device.motors[2].writeMicroseconds(throttle * enable_bit);
  device.motors[3].writeMicroseconds(throttle * enable_bit);
  
  delay(2500);
}


void loop() {

  int start = millis();
  float deltas[6];
  
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

  int adjustments[6] = {0, 0, 0, 0, 0, 0};
  device.heading2adjustments(deltas, adjustments);
  Serial.print("Adjustments: ");
  Serial.print("{x : ");
  Serial.print(adjustments[0]);
  Serial.print(", y : ");
  Serial.print(adjustments[1]);
  Serial.print(", z : ");
  Serial.print(adjustments[2]);
  Serial.print(", roll : ");
  Serial.print(adjustments[3]);
  Serial.print(", pitch : ");
  Serial.print(adjustments[4]);
  Serial.print(", yaw : ");
  Serial.print(adjustments[5]);
  Serial.println("}");
  poseX.setValue(device.pose.x);
  poseY.setValue(device.pose.y);
  poseZ.setValue(device.pose.z);
   
 //m_1.writeMicroseconds(value * enable_bit);
 //m_2.writeMicroseconds(value * enable_bit);
 //m_3.writeMicroseconds(value * enable_bit);
 //m_4.writeMicroseconds(value * enable_bit);
 
 //Serial.println(value);
 int t = millis();
 while( t - start < CYCLE_TIME)
  Serial.println("Under Cycled: try lowering cycle time");
  Serial.print("Previous Duration: ");
  Serial.println(t - start);
  t = millis();
}

void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}

void headingReceiver(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("Characteristic event, written: ");
}

void killReceiver(BLEDevice central, BLECharacteristic characteristic) {
  if(characteristic == '0'){
    Serial.print("Kill Switch Activated");
    enable_bit = 0;
  }
  else
    enable_bit = 1;
}
