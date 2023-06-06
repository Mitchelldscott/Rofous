#include <Arduino.h>
#include "utilities/vector.h"
#include "system_graph/graph_node.h"
#include "system_graph/system_graph.h"

#include "sensors/lsm6dsox.h"

int main() {
	while(!Serial){}

	Serial.println("=== Starting System Graph tests ===");
	
	float tmp[1] = {0.6};
	int imu_inputs[2] = {-1};
	int cmf_inputs[2] = {9, 10};

	SystemGraph sg;
	sg.add("LSM", 10, 9, 0, Vector<int>(imu_inputs, 1));
	sg.add("CMF", 9, 3, 1, Vector<int>(cmf_inputs, 2));
	sg.update_config(9, 0, Vector<float>(tmp, 1));
	sg.spin();
	sg.spin();
	sg.spin();
	sg.dump_all();

	Serial.println("=== Finished System Graph tests ===");
	
}