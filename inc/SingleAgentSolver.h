#pragma once
#include "Instance.h"

class LLNode // low-level node
{
public:
	int location;
	int g_val;
	int h_val = 0;
	LLNode* parent;
	int timestep = 0;
	bool in_openlist = false;
    bool is_goal = false;
	// the following is used to compare nodes in the OPEN list
	struct compare_node
	{
		// returns true if n1 > n2 (note -- this gives us *min*-heap).
		bool operator()(const LLNode* n1, const LLNode* n2) const
		{
            if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
            {
                if (n1->h_val == n2->h_val)
                {
                    return rand() % 2 == 0;   // break ties randomly
                }
                return n1->h_val >= n2->h_val;  // break ties towards smaller h_vals (closer to goal location)
            }
			return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
		}
	};  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)

	

	LLNode() : location(0), g_val(0), h_val(0), parent(nullptr), timestep(0), in_openlist(false) {}

	LLNode(int location, int g_val, int h_val, LLNode* parent, int timestep, int num_of_conflicts = 0, bool in_openlist = false) :
		location(location), g_val(g_val), h_val(h_val), parent(parent), timestep(timestep),
		in_openlist(in_openlist) {}

	inline int getFVal() const { return g_val + h_val; }
	void copy(const LLNode& other)
	{
		location = other.location;
		g_val = other.g_val;
		h_val = other.h_val;
		parent = other.parent;
		timestep = other.timestep;
        is_goal = other.is_goal;
	}
};


class SingleAgentSolver
{
public:
	int trial_idx;
	int start_location;
	int goal_location;
	vector<int> my_heuristic;  // this is the precomputed heuristic for this agent
	int compute_heuristic(int from, int to) const  // compute admissible heuristic between two locations
	{
		return max(get_DH_heuristic(from, to), instance.getManhattanDistance(from, to));
	}
	const Instance& instance;

	virtual Path findOptimalPath() = 0;
	virtual Path findSuboptimalPath() = 0;  // return the path and the lowerbound
	virtual string getName() const = 0;

	list<int> getNextLocations(int curr) const; // including itself and its neighbors

	int getStartLocation() const {return instance.start_locations[trial_idx]; }
	int getGoalLocation() const {return instance.goal_locations[trial_idx]; }

	///////// statistics & results
	float runtime = 0;
	float heuristics_time = 0;
	float path_finding_time = 0;
	float send_msg_time = 0;
	float rcv_msg_time = 0;
	float push_msg_time = 0;
	float barrier_time = 0;
	float expand_node_time = 0;
	int nproc = 1;
	uint64_t num_expanded = 0;
	uint64_t num_generated = 0;
	Path planned_path;
	int path_cost;

	void saveResults(const string &fileName, const string &instanceName) const;
	void savePaths(const string &fileName) const;


	SingleAgentSolver(const Instance& instance, int trial) :
		instance(instance), //agent(agent), 
		start_location(instance.start_locations[trial]),
		goal_location(instance.goal_locations[trial]),
		trial_idx(trial)
	{
		compute_heuristics();
	}

    virtual ~SingleAgentSolver() =default;

protected:
	int min_f_val; // minimal f value in OPEN
	// int lower_bound; // Threshold for FOCAL
	

	void compute_heuristics();
	int get_DH_heuristic(int from, int to) const { return abs(my_heuristic[from] - my_heuristic[to]); }
};

