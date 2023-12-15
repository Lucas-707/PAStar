#pragma once
#include "SingleAgentSolver.h"
#include "SpaceTimeAStar.h"
#include "mpi.h"

#define MAX_RECV_BUFF_SIZE 200000

class HDAStar: public SingleAgentSolver
{
public:
	// find path by time-space A* search
	// Returns a shortest path that satisfies the constraints of the give node  while
	// minimizing the number of internal conflicts (that is conflicts with known_paths for other agents found so far).
	// lowerbound is an underestimation of the length of the path in order to speed up the search.
	Path findOptimalPath();
	Path findSuboptimalPath();  // return the path and the lowerbound

	string getName() const { return "HDAStar"; }

	HDAStar(const Instance& instance, int agent, int nproc_, int pid_, string hash_func):
		SingleAgentSolver(instance, agent)
	{ 
		nproc = nproc_; 
		pid = pid_; 
		hash_f = hash_func;
		hash_table.resize(instance.map_size);
		if (hash_f == "mod") {
			for (int loc=0; loc< hash_table.size(); loc++) {
				hash_table[loc] = loc % nproc;
			}
		} else if (hash_f == "random") {
			for (int loc=0; loc< hash_table.size(); loc++) {
				hash_table[loc] = rand() % nproc;
			}
		} else if (hash_f == "grid") {
			for (int i=0; i< instance.num_of_rows / 8; i++) {
				for (int j=0; j< instance.num_of_cols / 8; j++) {
					int randint = rand() % nproc;
					for (int ii=0; ii< 8; ii++) {
						for (int jj=0; jj< 8; jj++) {
							hash_table[instance.linearizeCoordinate(i*8+ii,j*8+jj)] = randint;
						}
					}
				}
			}
		}
		// printf("finish init hash table, size=%d\n", hash_table.size());
		
	}

private:
	// define typedefs and handles for heap
	typedef pairing_heap< AStarNode*, compare<AStarNode::compare_node> > heap_open_t;
	heap_open_t open_list;
	int pid;
	bool dst_found = false;
	bool in_barrier_mode = false;
	int tag = 0;
    int num_sends = 0;
	string hash_f;
	vector<int> hash_table;

	MPI_Datatype MPI_Msg;
	struct msg {
        AStarNode node;
    }; 
	std::vector< std::vector<msg> > message_set;
	std::vector< std::vector<msg> > send_buffers;
	std::vector< MPI_Request* > send_requests;
	struct msg recv_buffer[MAX_RECV_BUFF_SIZE];
	

	// define typedef for hash_map
	typedef unordered_set<AStarNode*, AStarNode::NodeHasher, AStarNode::eqnode> hashtable_t;
	hashtable_t allNodes_table;

	// Updates the path datamember
	void updatePath(const LLNode* goal, vector<PathEntry> &path);
	inline AStarNode* popNode();
	inline void pushNode(AStarNode* node);
	void releaseNodes();

	void create_msg_mpi_datatype();
	void clear_message_set();
	int hash(const LLNode* node); //returns the owner of the node
	void send_message_set();
	int receive_message_set(); //returns number of messages received
	void add_msgs_to_open_list(int num_msgs_recvd);
	void add_local_node(AStarNode* next);
	struct msg create_msg(AStarNode* node);

};
