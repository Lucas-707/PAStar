#include "SingleAgentSolver.h"


list<int> SingleAgentSolver::getNextLocations(int curr) const // including itself and its neighbors
{
	list<int> rst = instance.getNeighbors(curr);
	return rst;
}


void SingleAgentSolver::compute_heuristics()
{
	struct Node
	{
		int location;
		int value;

		Node() = default;
		Node(int location, int value) : location(location), value(value) {}
		// the following is used to compare nodes in the OPEN list
		struct compare_node
		{
			// returns true if n1 > n2 (note -- this gives us *min*-heap).
			bool operator()(const Node& n1, const Node& n2) const
			{
				return n1.value >= n2.value;
			}
		};  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)
	};

	my_heuristic.resize(instance.map_size, MAX_TIMESTEP);

	// generate a heap that can save nodes (and an open_handle)
	boost::heap::pairing_heap< Node, boost::heap::compare<Node::compare_node> > heap;

	Node root(goal_location, 0);
	my_heuristic[goal_location] = 0;
	heap.push(root);  // add root to heap
	while (!heap.empty())
	{
		Node curr = heap.top();
		heap.pop();
		for (int next_location : instance.getNeighbors(curr.location))
		{
			if (my_heuristic[next_location] > curr.value + 1)
			{
				my_heuristic[next_location] = curr.value + 1;
				Node next(next_location, curr.value + 1);
				heap.push(next);
			}
		}
	}
}


void SingleAgentSolver::saveResults(const string &fileName, const string &instanceName) const
{
	std::ifstream infile(fileName);
	bool exist = infile.good();
	infile.close();
	if (!exist)
	{
		ofstream addHeads(fileName);
		addHeads << "runtime,nproc,path cost," <<
			"#node expanded,#node generated," <<
			"expand node time,send msg time," <<
			"rcv msg time,push msg time," <<
			"barreir time," <<
			"instance name,trial index" << endl;
		addHeads.close();
	}
	ofstream stats(fileName, std::ios::app);
	stats << runtime << "," << nproc << "," << path_cost << "," <<
		num_expanded << "," << num_generated << "," <<
		expand_node_time << "," << send_msg_time << "," <<
		rcv_msg_time << "," << push_msg_time << "," <<
		barrier_time << "," <<
		instanceName << "," << trial_idx << endl;
	stats.close();
}

void SingleAgentSolver::savePaths(const string &fileName) const
{
	std::ofstream output;
    output.open(fileName, std::ios::app);
	output << "Trial " << trial_idx << ": ";
	for (const auto & t : planned_path)
		output << "(" << instance.getRowCoordinate(t.location)
				<< "," << instance.getColCoordinate(t.location) 
				<< ")->";
	output << endl;
    output.close();
}