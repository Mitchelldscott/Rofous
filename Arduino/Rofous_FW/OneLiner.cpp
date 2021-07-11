#include "OneLiner.h"

OneLiner::OneLiner(){
	/*
		Default Initializer
	*/
	start = millis();
}

OneLinerMessage OneLiner::init(char* topic, int dtype){
	/*
		create a OneLinerMessage
	*/
	OneLinerMessage msg;
	msg.msgType = dtype;
	msg.topic = topic;
	msg.data = -1.0;
	msg.msg = "";

	return msg;
}

void OneLiner::publish(OneLinerMessage msg){
	/*
		publish a OneLinerMessage struct
	*/
	if (msg.topic != ""){
		Serial.print(msg.topic);Serial.write(',');
		Serial.print(msg.msgType);Serial.write(',');
		Serial.print(stamp());Serial.write(',');
		if(msg.data != -1.0){
			Serial.print(msg.data); 
			Serial.write('\n');
		}
		else if (msg.msg != ""){
			Serial.print(msg.msg);
			Serial.write('\n');
		}
	}
}

float OneLiner::stamp(){
	/*
		get time Stamp
		Returns
			timestamp: float - the current timestamp
	*/
	return (millis() - start) / 1000.0;
}