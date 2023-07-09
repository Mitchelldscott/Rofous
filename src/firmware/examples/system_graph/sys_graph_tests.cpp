#include <Arduino.h>
#include "utilities/timing.h"
#include "utilities/vector.h"
#include "system_graph/graph_node.h"
#include "system_graph/system_graph.h"

#include "sensors/lsm6dsox.h"

FTYK watch;

int main() {
	while(!Serial){}

	Serial.println("=== Starting System Graph tests ===");
	
	float tmp[1] = {0.6};
	int imu_inputs[2] = {-1};
	int cmf_inputs[2] = {9, 10};

	watch.set(0);
	SystemGraph sg;
	watch.print(0, "SG init");

	watch.set(1);
	sg.add("LSM", 10, 0, 1, imu_inputs);
	watch.print(1, "LSM Node init");

	watch.set(1);
	sg.add("CMF", 9, 1, 2, cmf_inputs);
	watch.print(1, "CMF Node init");

	watch.set(1);
	sg.update_config(9, 0, 1, tmp);
	watch.print(1, "Update config");

	for (int i = 0; i < 3; i++) {
		Serial.printf("Iteration: %i ===\n", i);
		watch.set(1);
		sg.spin();
		watch.print(1, "Run"); 
		Serial.flush();
	}
	sg.dump_all();

	Serial.print("Full test "); watch.print(0);

	Serial.println("=== Finished System Graph tests ===");
	
}