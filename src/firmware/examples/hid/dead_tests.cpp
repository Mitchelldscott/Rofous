// #include "unity.h"
#include "utilities/timing.h"
#include "utilities/assertions.h"
#include "robot_comms/hid_report.h"


HidReport outgoing_report;
HidReport incoming_report;

FTYK timer;


int packet_put_test() {
	int errors = 0;
	Serial.printf("\nPUT test:...\n");

	// execute the function in question
	timer.set(0);
	for (int i = 0; i < HID_REPORT_SIZE_BYTES; i++) {
		outgoing_report.put(i, 0xFF);
	}	
	timer.mark(0);

	// check the test
	for (int i = 0; i < HID_REPORT_SIZE_BYTES; i++) {
		errors += assert_eq<int>(0xFF, outgoing_report.get(i), "failed packet insert");
	}
	// byte* data[HID_REPORT_SIZE_BYTES];
	// outgoing_report.get(0, HID_REPORT_SIZE_BYTES, data);
	// return assert_eq()
	return errors;
}

int packet_get_test() {
	int errors = 0;
	Serial.printf("\nGET test:...\n");
	// insert semi-random values 
	for (int i = 0; i < HID_REPORT_SIZE_BYTES; i++) {
		outgoing_report.put(i, HID_REPORT_SIZE_BYTES - i);
	}

	// execute the function
	int8_t tmp[64];
	timer.set(0);
	for (int i = 0; i < HID_REPORT_SIZE_BYTES; i++) {
		tmp[i] = outgoing_report.get(i);
	}
	timer.mark(0);

	// check the output
	for (int i = 0; i < HID_REPORT_SIZE_BYTES; i++) {
		// Sum error of inserted values
		errors += assert_eq<int>(tmp[i], (HID_REPORT_SIZE_BYTES - i), "failed packet read");
	}

	return errors;
}

int packet_puts_gets_test() {
	Serial.printf("\nPUTS/GETS test:...\n");
	// insert semi-random values 
	byte tmp1[HID_REPORT_SIZE_BYTES];
	for (size_t i = 0; i < HID_REPORT_SIZE_BYTES; i++) {
		tmp1[i] = HID_REPORT_SIZE_BYTES - i;
	}

	timer.set(0);
	outgoing_report.rputs(tmp1, 0, HID_REPORT_SIZE_BYTES);
	timer.mark(0);

	// execute the function
	byte tmp2[HID_REPORT_SIZE_BYTES];
	timer.set(0);
	outgoing_report.rgets(tmp2, 0, HID_REPORT_SIZE_BYTES);
	timer.mark(0);

	// check the output
	return assert_eq<byte>(tmp1, tmp2, "Failed Puts-Gets", HID_REPORT_SIZE_BYTES);
}

int packet_int32_test() {
	Serial.printf("\nInt32 test:...\n");
	int32_t test_value = 913;
	// insert semi-random values 
	timer.set(0);
	for (int i = 0; i < HID_REPORT_SIZE_BYTES; i += 4) {
		outgoing_report.put_int32(i, test_value);
	}
	timer.mark(0);

	// check the values output from the function in question
	int tmp[HID_REPORT_SIZE_BYTES / 4];
	timer.set(0);
	for (int i = 0; i < HID_REPORT_SIZE_BYTES / 4; i++) {
		tmp[i] = outgoing_report.get_int32(4 * i);
	}
	timer.mark(0);

	// check the output
	return assert_eq<int>(tmp, int(test_value), "Failed int32_t test", HID_REPORT_SIZE_BYTES / 4);
}


int packet_float_test() {
	// int errors = 0;
	Serial.printf("\nFloat test:...\n");
	float test_value = 3.14159;

	// insert semi-random values 
	timer.set(0);
	for (int i = 0; i < HID_REPORT_SIZE_BYTES; i += 4) {
		outgoing_report.put_float(i, test_value);
	}
	timer.mark(0);
	outgoing_report.print();

	// check the values output from the function in question
	float tmp[HID_REPORT_SIZE_BYTES / 4];
	timer.set(0);
	for (int i = 0; i < HID_REPORT_SIZE_BYTES / 4; i++) {
		tmp[i] = outgoing_report.get_float(4 * i);
		Serial.printf("%f\t", tmp[i]);
		if ((i + 1) % 6 == 0 && i > 0) {
			Serial.println();
		} 
	}
	timer.mark(0);

	// check the output
	// for (int i = 0; i < HID_REPORT_SIZE_BYTES / 4; i++) {
	// 	// Sum error of inserted values
	// 	errors += assert_eq<float>(tmp[i], test_value, "Failed float test");
	// }

	return assert_eq<float>(tmp, test_value, "Failed float test", HID_REPORT_SIZE_BYTES / 4);
}

int run_hid_report_tests(void) {

	int errors = 0;
	errors += packet_put_test();
	errors += packet_get_test();
	errors += packet_puts_gets_test();
	errors += packet_int32_test();
	errors += packet_float_test();

	return errors;
}


// Runs once
int main() {
	// Serial.begin(1000000);

	// if (Serial)
	// Serial.println("-- TEENSY SERIAL START --");

	while (!Serial) {};
	Serial.println("Start comms tests");

	int errors = run_hid_report_tests();

	Serial.println("Finished tests");
	Serial.printf("%i failed\n", errors);

	return 0;
}


