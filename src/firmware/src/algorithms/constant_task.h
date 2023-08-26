#ifndef CONST_DRIVER_H
#define CONST_DRIVER_H

#include "task_manager/task.h"


class ConstTask: public Task {
	private:
		char key[3] = {'V', 'A', 'L'};
		int value;


	public:
		ConstTask() {
			dimensions.reset(TASK_DIMENSIONS);
			dimensions[INPUT_DIMENSION] = 0;
			dimensions[CONTEXT_DIMENSION] = 0;
			dimensions[OUTPUT_DIMENSION] = 1;
			dimensions[PARAMS_DIMENSION] = 1;

			reset();
		}

		void setup(Vector<float>* config) {
			value = (*config)[0];
			// analogWriteResolution(12);
		}

		void context(Vector<float>* context) {
			context->reset(dimensions[CONTEXT_DIMENSION]);
			context->push(value);
		}

		void reset() {
			value = 0;
		}

		void clear() {
			value = 0;
		}

		void run(Vector<float>* inputs, Vector<float>* outputs) {
			outputs->reset(1);
			(*outputs)[0] = float(value);
		}

		void print() {
			Serial.println("Constant Value task");
			Serial.printf("\tOutput: %i\n", value);
		}
};

#endif