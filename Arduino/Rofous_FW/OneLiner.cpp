#include "OneLiner.h"

OneLiner::OneLiner(){
	/*
		Default Initializer
	*/
	start = millis();

	msg->msg = "";
	msg->data = -1.0;
	msg->msgType = 0;
	msg->timestamp = 0.0;
	msg->topic = "Arduino_Serial";
}

OneLiner::OneLiner(char* topic){
	/*
		Paramaterized Initializer

		Params:
			topic: char* - set the topic to this
	*/
	start = millis();

	msg->msg = "";
	msg->data = -1.0;
	msg->msgType = 0;
	msg->topic = topic;
	msg->timestamp = 0.0;
}

OneLiner::~OneLiner(){
	/*
		Destructor
	*/
	delete msg;
}

void OneLiner::setTopic(char* new_topic){
	/*
		setter for msg->topic
	*/
	msg->topic = new_topic;
}

char* OneLiner::getTopic(){
	/*
		getter for msg->topic

		Returns
			topic: char* - msg->topic
	*/
	return msg->topic;
}

float OneLiner::stamp(){
	/*
		get time Stamp
		Returns
			timestamp: float - the current timestamp
	*/
	return (millis() - start) / 1000.0;
}

int OneLiner::write(){
	/*
		Write the default mesg to the serial port
	*/
	if (msg->topic != ""){
		Serial.print(msg->topic);Serial.write(',');
		Serial.print(msg->msgType);Serial.write(',');
		Serial.print(msg->timestamp);Serial.write(',');
		if(msg->data != -1.0){
			Serial.print(msg->data); 
			Serial.write('\n');
		}
		else if (msg->msg != ""){
			Serial.print(msg->msg);
			Serial.write('\n');
		}
	}
	return 1;
}

int OneLiner::write(char* message){
	/*
		Write a fresh mesg to the serial port

		Params
			message: char* - the message to publish
	*/
	Serial.print(message);Serial.write('\n');
	return 1;
}

int OneLiner::writeMsg(float data){
	/*
		Build a float mesg and send to the serial port
		using the default topic

		Params
			data: float - the data to publish
	*/
	msg->msgType = 1;
	msg->data = data;
	msg->timestamp = stamp();
	
	return write();
}

int OneLiner::writeMsg(char* message){
	/*
		Build a char* mesg and send to the serial port
		using the default topic

		Params
			message: char* - the message to publish
	*/
	msg->msgType = 0;
	msg->msg = message;
	msg->timestamp = stamp();
	
	return write();
}

int OneLiner::writeMsg(char* topic, char* data){
	/*
		Build a char* mesg and send to the serial port
		using the default topic

		Params
			topic: char* - the topic to publish to
			data: char* - the data to publish
	*/
	Serial.print(topic);
	Serial.print(",0,"); 
	Serial.print(stamp()); 
	Serial.print(","); 
	Serial.print(data); 
	Serial.write('\n');
	
	return 1;
}

int OneLiner::writeMsg(char* topic, float data){
	/*
		Build a float mesg and send to the serial port
		using the default topic

		Params
			topic: char* - the topic to publish to
			data: float - the data to publish
	*/
	Serial.print(topic);
	Serial.print(",1,");  
	Serial.print(stamp()); 
	Serial.print(","); 
	Serial.print(data); 
	Serial.write('\n');
	
	return 1;
}

int OneLiner::writeMsg(char* topic, float* data, int l){
	/*
		Build a float* mesg and send to the serial port
		using the default topic

		Params
			topic: char* - the topic to publish to
			data: float* - the data to publish
			l: int - the length of the array
	*/
	Serial.print(topic);
	Serial.print(",2,");
	Serial.print(stamp());
	Serial.print(",");

	for (int i=0; i < l - 1; i++){
		Serial.print(data[i]);
		Serial.write(',');
	}

	Serial.print(data[l -1]);
	Serial.print('\n');	
	return 1;
}