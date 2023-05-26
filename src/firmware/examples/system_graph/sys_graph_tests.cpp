#include <Arduino.h>
#include "system_graph/vector.h"
#include "system_graph/graph_node.h"
#include "system_graph/system_graph.h"

#include "sensors/lsm6dsox.h"

int main() {
	while(!Serial){}

	Serial.println("=== Starting Graph Node tests ===");

	SystemGraph sg;
	// GraphNode<LSM6DSOX> node(0, 9, Vector<float>(1));

	Serial.println("=== Finished Graph Node tests ===");
	
}