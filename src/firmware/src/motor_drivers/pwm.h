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
			dimensions[CONTEXT_DIMENSION] = 1;
			dimensions[OUTPUT_DIMENSION] = 1;
			dimensions[PARAMS_DIMENSION] = 1;

			reset();
		}

		void setup(Vector<float>* config) {
			output = 128;
			pin = (*config)[0];
			pinMode(pin, OUTPUT);
			analogWrite(pin, output);
			// analogWriteResolution(12);
		}

		void context(Vector<float>* context) {
			context->reset(dimensions[CONTEXT_DIMENSION]);
			context->push(output);
		}

		void reset() {
			output = 0;
		}

		void clear() {
			output = 0;
		}

		void run(Vector<float>* inputs, Vector<float>* outputs) {
			analogWrite(pin, int((*inputs)[0]));
			outputs->reset(1);
			(*outputs)[0] = float(output);
		}

		void print() {
			Serial.println("PWM Driver");
			Serial.printf("\tPin: %i\n", pin);
			Serial.printf("\tOutput: %i\n", output);
		}
};

#endif