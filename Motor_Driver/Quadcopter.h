/*
  Target Device: Arduino Nano IOT 33

  This file is for organizing devices that track, sense and control 
  movement/orientation of a quadcopter.
  
  created 21 July, 2020
  by Mitchell Scott

  ~This code is an original implementation~
*/
#include <Servo.h>

#ifndef QUADCOPTER_H
#define QUADCOPTER_H


struct Pose3D{
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
};


/*
 *   This function is a compliment to the Pose3D object. 
 *  It is used to update a 3-dimensional pose (x,y,z,roll,pitch,theta) 
 *  using 6-DOF accel/gyro. The cycle time is assumed to be 1 for ease of
 *  development. In an actual implementtion this needs to be included in
 *  as an arguement/global.
 *  ~Params
 *    - Pose3D: struct of coordinates/angles -> {'x':0, 'y':0, 'z':0, 'roll':0, 'pitch':0, 'yaw':0} float
 *    - deltas: accel/gyro readings -> [0, 0, 0, 0, 0, 0] float
 *    - cycle_length time measurement between each measurement/adjustment -> float
 *  ~Return
 *    - None
 */
void update_pose(Pose3D* pose, float deltas[6], float cycle_length){
  pose->x += deltas[0] * cycle_length;
  pose->y += deltas[1] * cycle_length;
  pose->z += deltas[2] * cycle_length;
  pose->roll += deltas[3] * cycle_length;
  pose->pitch += deltas[4] * cycle_length;
  pose->yaw += deltas[5] * cycle_length;
}


class Quadcopter {
  public:
    Servo motors[4];
    Pose3D pose;
    float heading[3] = {0.0, 0.0, 0.0};
    int throttle;
    
    Quadcopter(int cycle_time, int pin1, int pin2, int pin3, int pin4){
      pose = {0, 0, 0, 0, 0, 0};
      motors[0].attach(pin1);
      motors[1].attach(pin2);
      motors[2].attach(pin3);
      motors[3].attach(pin4);
      throttle = 1000;
    }
    

    /*
    *    This Function is used as a helper to determine throttle adjustments
    *  from a purely directional heading. The input heading is in the form (theta, phi).
    *  This gives us everything we need to find x, y, z, roll, pitch and yaw adjustments.
    *  ~Params
    *    - theta: the rotational offset in xy plane where 0 is the x-axis -> float
    *    - phi: rotational offset of the z-axis where 0 is the z-axis -> float
    *    - e_bit: an enabler boolean to prevent movement -> boolean
    *  ~Return
    *    - None
    */
    void spherical2cartesian(int t, int p, bool e_bit){
      float theta = radians(t);
      float phi = radians(p);
      heading[0] = cos(theta)*sin(phi) * e_bit;
      heading[1] = sin(theta)*sin(phi) * e_bit;
      heading[2] = cos(phi) * e_bit;
    }

    
    /*
     *  This function gets the exact motor adjustments for a given heading
     * and state (dPose). The principle is when the device is stationary the heading is 
     * the ony thing that will affect adjustments. However, when the device is already in 
     * motion, it must take into account its current accel/gyro when finding the adjustment.
     * The current theory is that by splitting the device into two axis I can determine
     * based on the accel/gyro that if adevice wants to adjust its roll, the adjustment 
     * is added to the positive side of the axis and subtracted from the negative.
     * The adjustments that the device watches is pitch, roll, altitude (z) and yaw (x,y don't make a difference).
     * ~Params
     *  - deltas: most recent Accel/Gyro readings -> [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] float
     *  - adjustments: The adjustments for heading, pitch, roll, z and yaw -> [0.0, 0.0, 0.0, 0.0] float
     * ~Return
     *  - None
     */
    void heading2adjustments(float deltas[6], int adjustments[4]){
      adjustments[0] = int(heading[0] - (deltas[0] + deltas[4]));
      adjustments[1] = int(heading[1] - (deltas[1] + deltas[3]));
      //Serial.println(heading[2]);
      //Serial.println(deltas[2]);
      adjustments[2] = int(heading[2] - deltas[2]);
      adjustments[3] = int(-deltas[5]);
    }
    /*
     *  This function is a simple motor driver.
     * Given a throttle and stability adjustments, the functions 
     * sets a pwm signal to each motor.
     * ~Params
     *  - pose_adj: the 4 adjustment values -> [0.0, 0.0, 0.0, 0.0] float
     *  - enable_bit: a boolean value that enables adjustments
     *  ~Return
     *  - None
     */
    int adjust_motor_speeds(int pose_adj[4], boolean enable_bit){
      //Serial.print("Active Throttle: ");
      motors[0].writeMicroseconds(throttle + pose_adj[0] + pose_adj[1] + pose_adj[2] + pose_adj[3] * enable_bit);
      //Serial.print(throttle + pose_adj[0] + pose_adj[1] + pose_adj[2] + pose_adj[3] * enable_bit);
      //Serial.print("\t");
      motors[1].writeMicroseconds(throttle + pose_adj[0] - pose_adj[1] + pose_adj[2] - pose_adj[3] * enable_bit);
      //Serial.print(throttle + pose_adj[0] - pose_adj[1] + pose_adj[2] - pose_adj[3] * enable_bit);
      //Serial.print("\t");
      motors[2].writeMicroseconds(throttle - pose_adj[0] - pose_adj[1] + pose_adj[2] - pose_adj[3] * enable_bit);
      //Serial.print(throttle - pose_adj[0] - pose_adj[1] + pose_adj[2] - pose_adj[3] * enable_bit);
      //Serial.print("\t");
      motors[3].writeMicroseconds(throttle - pose_adj[0] + pose_adj[1] + pose_adj[2] + pose_adj[3] * enable_bit);
      //Serial.println(throttle - pose_adj[0] + pose_adj[1] + pose_adj[2] + pose_adj[3] * enable_bit);
      throttle = ((throttle + pose_adj[0] + pose_adj[1] + pose_adj[2] + pose_adj[3] * enable_bit) + 
                   (throttle + pose_adj[0] - pose_adj[1] + pose_adj[2] - pose_adj[3] * enable_bit) +
                    (throttle - pose_adj[0] - pose_adj[1] + pose_adj[2] - pose_adj[3] * enable_bit) + 
                      (throttle - pose_adj[0] + pose_adj[1] + pose_adj[2] + pose_adj[3] * enable_bit)) / 4;
    }
};

#endif
