#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

#include "utilities/timing.h"
#include "task_manager/task.h"


class PwmDriver: public Task {
	private:
		char key[3] = {'P', 'W', 'M'};
		int pin;
		int output;


	public:
		PwmDriver() {
			dimensions.reset(TASK_DIMENSIONS);
			dimensions[INPUT_DIMENSION] = 1;
			dimensions[PARAM_DIMENSION] = 1;
			dimensions[OUTPUT_DIMENSION] = 1;

			reset();
		}

		void setup(Vector<float>* config) {
			pin = (*config)[0];
			pinMode(pin, OUTPUT);
			// analogWriteResolution(12);
		}

		void reset() {
			output = 0;
		}

		void clear() {
			output = 0;
		}

		void run(Vector<float>* inputs, Vector<float>* outputs) {
			output = int((*inputs)[0]);
			analogWrite(pin, output);
			(*outputs)[0] = float(output);
		}

		void print() {
			Serial.println("PWM Driver");
			Serial.printf("\tPin: %i\n", pin);
			Serial.printf("\tOutput: %i\n", output);
		}
};

#endif