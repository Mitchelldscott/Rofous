#include "MPU6050.h"

MPU6050 mpu(1);

void printVector(Vector* vect, String title)
{
  /*
   * A helper for printing Vectors to the Serial Console
   */
  Serial.print(title + " (");
  Serial.print(vect->X); Serial.print(", "); Serial.print(vect->Y); Serial.print(", "); Serial.print(vect->Z); Serial.println(")");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  Serial.println("Initialized Program!");

  mpu.begin();
}

void loop() {

  Vector rawA, rawG;
  mpu.readRawAccel(&rawA);
  mpu.readRawGyro(&rawG);
  mpu.test_WhoAmI();
  
  Serial.print("WhoAmI test: "); Serial.println(mpu.getState());
  printVector(&rawA, "MPU6050 Accel Raw: ");
  printVector(&rawG, "MPU6050 Gyro Raw: ");
}
