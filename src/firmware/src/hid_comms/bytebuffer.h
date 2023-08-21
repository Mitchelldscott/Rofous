/********************************************************************************
 * 
 *      ____                     ____          __           __       _          
 *	   / __ \__  __________     /  _/___  ____/ /_  _______/ /______(_)__  _____
 *	  / / / / / / / ___/ _ \    / // __ \/ __  / / / / ___/ __/ ___/ / _ \/ ___/
 *	 / /_/ / /_/ (__  )  __/  _/ // / / / /_/ / /_/ (__  ) /_/ /  / /  __(__  ) 
 *	/_____/\__, /____/\___/  /___/_/ /_/\__,_/\__,_/____/\__/_/  /_/\___/____/  
 *	      /____/                                                                
 * 
 * 
 * 
 ********************************************************************************/

#include <Arduino.h>

#ifndef BYTEBUFFER_H
#define BYTEBUFFER_H

typedef union
{
	float number;
	byte bytes[4];
} FLOATBYTE_t;

/*
	buffer of n bytes
	read and write its data.
*/

template <int buffer_size> class ByteBuffer {
	private:
		float timestamp;
		byte data[buffer_size];

	public:

		void print(){
			/*
				  Display function for HID packets
			*/
			Serial.println("\n\tByteBuffer =====");
			if (buffer_size > 16) {
				for (int i = 0; i < buffer_size - 15; i += 16){
					Serial.printf("\t[%d]\t\t%X, %X, %X, %X, %X, %X, %X, %X, %X, %X, %X, %X, %X, %X, %X, %X\n", 
								i,
								data[i], data[i+1], data[i+2], data[i+3],
								data[i+4], data[i+5], data[i+6], data[i+7], 
								data[i+8], data[i+9], data[i+10], data[i+11],
								data[i+12], data[i+13], data[i+14], data[i+15]);
				}
			}
			else {
				Serial.print("\t\t");
				for (int i = 0; i < buffer_size; i++) {
					Serial.printf("%X, ", data[i]);
				}
				Serial.println();
			}
		}

		// clear all data in the packet (zero)
		void clear() {
			memset(data, 0, buffer_size);
		}

		// Getters/Setters
		byte* buffer() {
			return data;
		}

		template <typename T> void put(int index, T value) {
			memcpy(&data[index], &value, sizeof(T));
		}

		template <typename T> void put(int index, int n, T* values){
			for (int i = 0; i < n; i++) {
				put(index + (i * sizeof(T)), values[i]);
			}
			// memcpy(&data[index], values, n * sizeof(T));
		}

		template <typename T> T get(int index) {
			T value;
			memcpy(&value, &data[index], sizeof(T));
			return value;
		}

		template <typename T> void get(int index, int n, T* buffer) {
			for (int i = 0; i < n; i++) {
				buffer[i] = get<T>(index + (i * sizeof(T)));
			}
			// memcpy(&data[index], buffer, n *sizeof(T));
		}
};

#endif