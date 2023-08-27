#ifndef SIN_DRIVER_H
#define SIN_DRIVER_H

#include "utilities/timing.h"
#include "task_manager/task.h"


class SinTask: public Task {
	private:
		char key[3] = {'V', 'A', 'L'};
		float frequency;
		float amplitude;
		float shift;
		FTYK timers;


	public:
		SinTask() {
			dimensions.reset(TASK_DIMENSIONS);
			dimensions[INPUT_DIMENSION] = 0;
			dimensions[CONTEXT_DIMENSION] = 0;
			dimensions[OUTPUT_DIMENSION] = 1;
			dimensions[PARAMS_DIMENSION] = 3;

			reset();
		}

		void setup(Vector<float>* config) {
			frequency = (*config)[0];
			amplitude = (*config)[1];
			shift = (*config)[2];
			timers.set(0);
			// print();
			// analogWriteResolution(12);
		}

		void context(Vector<float>* context) {
			context->reset(dimensions[CONTEXT_DIMENSION]);
		}

		void reset() {
			frequency = 0;
			amplitude = 0;
			shift = 0;
			timers.set(0);
		}

		void clear() {
			frequency = 0;
			amplitude = 0;
			shift = 0;
			timers.set(0);
		}

		void run(Vector<float>* inputs, Vector<float>* outputs) {
			(*outputs)[0] = (amplitude * sin(frequency * timers.secs(0))) + shift;
		}

		void print() {
			Serial.println("Constant Sin task");
			Serial.printf("\tFrequency: %i\n", frequency);
			Serial.printf("\tAmplitude: %i\n", amplitude);
		}
};

#endif