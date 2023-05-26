#include <Arduino.h>
#include "system_graph/vector.h"
#include "system_graph/graph_node.h"

#include "sensors/lsm6dsox.h"

int main() {
	while(!Serial){}

	Serial.println("=== Starting Graph Node tests ===");

	GraphNode<LSM6DSOX> node(0, 9, Vector<float>(1));

	node.run_proc(Vector<Vector<float>>(1));

	node.run_proc(Vector<Vector<float>>(1));

	node.run_proc(Vector<Vector<float>>(1));

	node.print_proc();

	node.print_output();

	Serial.println("=== Finished Graph Node tests ===");
	
}