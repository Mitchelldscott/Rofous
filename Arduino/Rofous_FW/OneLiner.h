#include <Arduino.h>

using namespace std;

#ifndef __OneLiner_H__
#define __OneLiner_H__

struct OneLinerMessage{
  float data;
  int msgType;
  char* topic;
  char* msg;
};

class OneLiner
{
  /*
    OneLiner is a simple class for arduino 
  that makes writing messages to rofous_ros
  clean and easy. These messages are handled
  by the arduino_reader package.
  */
  private:
    unsigned long start;

  public:
    OneLiner();                        // Default init

    float stamp();                    // Return curent timestamp
    void publish(OneLinerMessage);    // publish a OneLinerMessage
    OneLinerMessage init(char*, int); // Create a message instance
};

# endif
