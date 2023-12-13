#ifndef GASTAR_H
#define GASTAR_H

#include "SingleAgentSolver.h"
#include "SpaceTimeAStar.h"

#define HASH_SIZE  (1024 * 1024)
#define HASH_FUNS 128

#define THREADS_PER_BLOCK  1024
#define BLOCKS 16
#define RESULT_LEN (1024 * 1024)


class GAStar: public SingleAgentSolver
{
public:
	// find path by time-space A* search
	// Returns a shortest path that satisfies the constraints of the give node  while
	// minimizing the number of internal conflicts (that is conflicts with known_paths for other agents found so far).
	// lowerbound is an underestimation of the length of the path in order to speed up the search.
	Path findOptimalPath();
	Path findSuboptimalPath();  // return the path and the lowerbound

	string getName() const { return "GAStar"; }

	GAStar(const Instance& instance, int agent):
		SingleAgentSolver(instance, agent){}

private:
	// define typedef for hash_map
	// typedef unordered_set<AStarNode*, AStarNode::NodeHasher, AStarNode::eqnode> hashtable_t;

	// Updates the path datamember
	void updatePath(const LLNode* goal, vector<PathEntry> &path);
	inline AStarNode* popNode();
	inline void pushNode(AStarNode* node);
	void releaseNodes();
};

#endif