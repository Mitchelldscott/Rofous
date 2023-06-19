#ifndef BUFF_LOGGERS
#define BUFF_LOGGERS
/*
	Plans for this is to become like unity macros
	- used for assertions and debug printing
	- need to support unit testing
	- if it gets big enough start making a fancy serial layer (PC side)
*/

int int_eq(int, int, String);
int ints_eq(int, int*, int, String);
int ints_eq(int*, int*, int, String);
int byte_eq(int, int, String);
int bytes_eq(byte, byte*, int, String);
int bytes_eq(byte*, byte*, int, String);
int float_eq(float, float, String);
int floats_eq(float*, float*, int, String);
int float_neq(float, float, String);
int float_nan(float);
int float_leq(float, float, String);
int float_geq(float, float, String);

void timer_print(uint32_t, String);

#endif