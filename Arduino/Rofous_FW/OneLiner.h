#include <Arduino.h>

using namespace std;

#ifndef __OneLiner_H__
#define __OneLiner_H__

struct OneLinerMessage{
  char* msg;
  float data;
  int msgType;
  char* topic;
  float timestamp;
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
    OneLinerMessage* msg;

  public:
    OneLiner();           // Default init
    OneLiner(char*);      // Init with default topic
    
    ~OneLiner();

    float stamp();         // Return timestamp
    char* getTopic();      // get default topic
    void setTopic(char*);  // Set default topic


    ////////    Write Messages via Serial   ////////
    int write();
    int write(char*);
    int writeMsg(float);
    int writeMsg(char*);
    int writeMsg(float*, int);
    int writeMsg(char*, float);
    int writeMsg(char*, char*);
    int writeMsg(char*, float*, int);
    
};

# endif
