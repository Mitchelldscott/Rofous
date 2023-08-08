#include <Arduino.h>

#ifndef BUFF_HIDREPORT_H
#define BUFF_HIDREPORT_H

#define HID_REPORT_SIZE_BYTES 64

typedef union
{
	float number;
	byte bytes[4];
} FLOATBYTE_t;

/*
	HID specific buffer of 64 bytes
	read and write its data.
*/

class HidReport {
	private:
		byte data[HID_REPORT_SIZE_BYTES];

	public:
		// constructor
		HidReport();

		// for debug, print the packet with serial.print
		void print();
		// clear all data in the packet
		void clear();

		// Getters/Setters
		byte get(int);
		void get(int, int, byte*);

		void put(int, byte);
		void put(int, int, byte*);

		int32_t get_int32(int);
		void put_int32(int, int32_t);

		float get_float(int);
		void put_float(int, float);

		void get_chars(int, int, char*);
		void put_chars(int, char*);

		void rgets(byte*, int, int);
		void rputs(byte*, int, int);

		// fill data with the available HID packet, (wait if none?)
		int8_t read();
		// send data as HID packet
		int8_t write();
};

#endif