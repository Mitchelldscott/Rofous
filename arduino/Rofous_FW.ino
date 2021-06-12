#include <Arduino_LSM6DS3.h>


#define NAME "Rofous_FW"

float pose[6];
float gyro[3];
float accel[3];
String asset_cmd = "";

void writePhrase(String topic)
{
  Serial.print(topic);Serial.write('\n');
}

void writePhrase(String topic, String data)
{
    Serial.print(topic); Serial.write(','); Serial.write('0'); Serial.write(','); 
    Serial.print(data); Serial.write('\n');
}

void writePhrase(String topic, float data)
{
    Serial.print(topic); Serial.write(','); Serial.write('1'); Serial.write(',');
    Serial.print(data);Serial.write('\n');
}

void writePhrase(String topic, float* data, int l)
{
  Serial.print(topic); Serial.write(','); Serial.write('2'); Serial.write(',');
  for (int i=0; i < l - 1; i++)
  {
    Serial.print(data[i]); Serial.write(',');
  }
  Serial.print(data[l-1]); Serial.write('\n');
}

void serialEvent() {
  asset_cmd = "";
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n')
      break;

    asset_cmd += inChar;
  }
  
  if (asset_cmd == "KILL")
    writePhrase("SUDO");
  if (asset_cmd == "WHOAMI")
    writePhrase(NAME, NAME);
}

void readIMUFloat(float* accel, float* gyro){
  if (IMU.accelerationAvailable())
    IMU.readAcceleration(accel[0], accel[1], accel[2]);

  if (IMU.gyroscopeAvailable())
    IMU.readGyroscope(gyro[0], gyro[1], gyro[2]);
}

void readIMU(float* pose){
  if (IMU.accelerationAvailable())
    IMU.readAcceleration(pose[0], pose[1], pose[2]);

  if (IMU.gyroscopeAvailable())
    IMU.readGyroscope(pose[3], pose[4], pose[5]);
}

void publishIMUFloat()
{
  writePhrase("Xaccel", accel[0]);
  writePhrase("Yaccel", accel[1]);
  writePhrase("Zaccel", accel[2]);
  writePhrase("Xgyro", gyro[0]);
  writePhrase("Ygyro", gyro[1]);
  writePhrase("Zgyro", gyro[2]);
}

void publishIMU(float* pose)
{
  writePhrase("IMU", pose, 6);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    writePhrase("Error", "IMU not initialized");
    while (1);
  }

  writePhrase(NAME, NAME);
}

void loop() {
  readIMU(pose);
  publishIMU(pose);
  delay(200);
  serialEvent();
  
}
