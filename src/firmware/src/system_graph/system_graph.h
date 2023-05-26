#ifndef SYS_GRAPH_OBJ
#define SYS_GRAPH_OBJ

#include "system_graph/vector.h"
#include "system_graph/process.h"
#include "system_graph/graph_node.h"

#define MAXIMUM_GRAPH_NODES 10

class SystemGraph {
	private:
		GraphNode<DriverTypes> nodes[MAXIMUM_GRAPH_NODES];
		Vector<Vector<int>> inputs;
		Vector<Vector<int>> outputs;

	public:
		SystemGraph();
		// ~SystemGraph();
		void add(GraphNode<DriverTypes>, Vector<int>, Vector<int>);
		void add(int, int, Vector<float>, Vector<int>, Vector<int>);
};

#endif