#include <Arduino.h>
#include "utilities/timing.h"
#include "utilities/vector.h"
#include "syncor/syncor_node.h"
#include "syncor/syncor.h"

#include "sensors/lsm6dsox.h"

FTYK watch;

int main() {
	while(!Serial){}

	Serial.println("=== Starting System Graph tests ===");
	
	float tmp[1] = {0.6};
	int imu_input;
	int cmf_inputs[2] = {9, 10};

	watch.set(0);
	SynCor sc;
	watch.print(0, "SynCor init");

	watch.set(1);
	sc.add("LSM", 10, 0, 0, &imu_input);
	watch.print(1, "LSM Node init");

	watch.set(1);
	sc.add("CMF", 9, 1, 2, cmf_inputs);
	watch.print(1, "CMF Node init");

	watch.set(1);
	sc.update_config(9, 0, 1, tmp);
	watch.print(1, "Update config");

	for (int i = 0; i < 3; i++) {
		// Serial.printf("Iteration: %i ===\n", i);
		watch.set(1);
		sc.spin();
		watch.print(1, "Run"); 
		// Serial.flush();
	}
	Serial.print("Full test "); watch.print(0);
	sc.dump_all();
	Serial.println("=== Finished System Graph tests ===");
	
}