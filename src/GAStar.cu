#include "GAStar.h"
#include "list.h"
#include "heap.h"

__global__ void clear_open_list(llist *S);
__global__ void fill_open_list(int k);
__global__ void deduplicate(llist *T);
__global__ void push_to_queues(int k, heap **open_list, llist *S, int off);

__device__ unsigned int jenkins_hash(int j, AStarNode *node);
__device__ int calculate_index();

__device__ int calculate_index() {
	return  blockIdx.x * blockDim.x + threadIdx.x;
}

__device__ int total_open_list_size = 0;
__device__ int found = 0;
__device__ int out_of_memory = 0;

void GAStar::updatePath(const LLNode* goal, vector<PathEntry> &path)
{
    const LLNode* curr = goal;
    if (curr->is_goal)
        curr = curr->parent;
    path.reserve(curr->g_val + 1);
    while (curr != nullptr)
    {
        path.emplace_back(curr->location);
        curr = curr->parent;
    }
    std::reverse(path.begin(),path.end());
}


Path GAStar::findOptimalPath()
{
    return findSuboptimalPath();
}

__device__ unsigned int jenkins_hash(int j, AStarNode *node) {
	char c;
	unsigned long hash = (j * 10000007);
	while (c = node->location++) {
		hash += c;
		hash += hash << 10;
		hash ^= hash >> 6;
	}
	hash += hash << 3;
	hash ^= hash >> 11;
	hash += hash << 15;
	return hash;
}

// find path by time-space A* search
// Returns a bounded-suboptimal path that satisfies the constraints of the give node  while
// minimizing the number of internal conflicts (that is conflicts with known_paths for other agents found so far).
// lowerbound is an underestimation of the length of the path in order to speed up the search.
Path GAStar::findSuboptimalPath()
{
    Path path;
    num_expanded = 0;
    num_generated = 0;

	int k = THREADS_PER_BLOCK * BLOCKS;


	cudaMalloc(&allNodes_table, HASH_SIZE * sizeof(AStarNode*));
	cudaMemset(allNodes_table, 0, HASH_SIZE * sizeof(AStarNode*));
	// priority queues of open lists (Q)
	heap **open_list = heaps_create(k);
	llist *S = list_create(1024 * 1024);
	int total_open_list_size_cpu;
	int found_cpu;
	int out_of_memory_cpu;

	auto start = new AStarNode(start_location, 0, compute_heuristic(start_location, goal_location), nullptr, 0, 0);
	pushNode(open_list[0], start)
	atomicAdd(&total_open_list_size, 1);
	int step = 0;

    do {
		clear_open_list<<<1, 1>>>(S);
		cudaDeviceSynchronize();
		fill_open_list<<<BLOCKS, THREADS_PER_BLOCK>>>(k);
		cudaMemcpyFromSymbol(&found_cpu, found, sizeof(int));
		cudaMemcpyFromSymbol(&out_of_memory, found, sizeof(int));
		if (found_cpu) break;
		if (out_of_memory_cpu) break;
		cudaDeviceSynchronize();
		deduplicate<<<BLOCKS, THREADS_PER_BLOCK>>>(S);
		cudaDeviceSynchronize();
		push_to_queues<<<1, THREADS_PER_BLOCK>>>(k, step) ;
		cudaDeviceSynchronize();
		cudaMemcpyFromSymbol(&total_open_list_size_cpu, total_open_list_size, sizeof(int));
		step++;
	} while (total_open_list_size_cpu > 0);


	// heaps_destroy(open_list, k);
	// HANDLE_RESULT(cudaFree(allNodes_table));
	cudaDeviceSynchronize();

    // // generate start and add it to the OPEN & FOCAL list
    // auto start = new AStarNode(start_location, 0, compute_heuristic(start_location, goal_location), nullptr, 0, 0);

    // pushNode(start);
    // allNodes_table.insert(start);
    // min_f_val = (int) start->getFVal();
    // lower_bound = int(w * min_f_val));
	releaseNodes();

	return path;

}

void GAStar::expandNode(AStarNode *next, heap *open_list, llist S){
	auto next_locations = instance.getNeighbors(curr->location);
	next_locations.emplace_back(curr->location);
	for (int next_location : next_locations)
	{
		int next_timestep = curr->timestep + 1;
		// compute cost to next_id via curr node
		int next_g_val = curr->g_val + 1;
		int next_h_val = compute_heuristic(next_location, goal_location);
		
		// generate (maybe temporary) node
		auto next = new AStarNode(next_location, next_g_val, next_h_val,
									curr, next_timestep);

		list_insert(S, next);
		// delete(next);  // not needed anymore -- we already generated it before
	}
}


inline AStarNode* GAStar::popNode(heap *open_list)
{
    auto node = open_list.top(); open_list.pop();
    // open_list.erase(node->open_handle);
    node->in_openlist = false;
    num_expanded++;
    return node;
}


inline void GAStar::pushNode(heap *open_list, AStarNode* node)
{
    node->open_handle = open_list.push(node);
    node->in_openlist = true;
    num_generated++;
}

void GAStar::releaseNodes()
{
	// TODO: modify
    // open_list.clear();
    for (auto node: allNodes_table)
        delete node;
    allNodes_table.clear();
}

__global__ void clear_open_list(llist *S) {
	list_clear(S);
}

__global__ void fill_open_list(int k, Instance* inst) {
	auto *bestNode = NULL;
	int index = calculate_index();
	if (index == 0) steps++;

	if (!open_list[index].empty()){
		auto* curr = popNode(open_list[index]);
		atomicSub(&total_open_list_size, 1);
		if (curr->location == goal_location) {
			if (bestNode == NULL || (curr->g_val + curr->h_val) < (bestNode->g_val + bestNode->h_val)) {
				bestNode = copy(*curr);
				if (bestNode != NULL && (bestNode->g_val + bestNode->h_val) <= heaps_min(open_list, k)) {
					// Found a better path, update found and return the path
					int found_before = atomicCAS(&found, 0, 1);
					if (found_before == 1) return;
					updatePath(bestNode, path);
					return;
				}
			}
		}
	}
	
	// Expand S
	int* neighbors = malloc(4 * sizeof(int));
	int* n_size;
	getNeighbors(curr, neighbors, n_size, inst);
	for (int j = 0; j<n_size; j++) {
		int delta = states_delta(q->node, my_expand_buf[j]);
		state *new_state = state_create(my_expand_buf[j], -1, q->g + delta, q, states_pool, nodes_pool, state_len);
		if (new_state == NULL) return;
		list_insert(S, new_state);
	}
	
}

__global__ void deduplicate(llist *T) {
	int index = calculate_index();
	int z = 0;
	AStarNode *t1 = list_get(T, index);
	for (int j = 0; j < HASH_FUNS; j++) {
		assert(t1 != NULL);
		auto el = allNodes_table[jenkins_hash(j, t1) % HASH_SIZE];
		if (el == NULL || cuda_str_eq(t1, el)) {
			z = j;
			break;
		}
	}
	t1 = (AStarNode*)atomicExch((unsigned long long*)&(allNodes_table[jenkins_hash(z, t1) % HASH_SIZE]), (unsigned long long)t1);
	if (t1 != NULL && t1 == list_get(T, index) &&
			((list_get(T, index)->g_val + list_get(T, index)->h_val) >= (t1->g_val + t1->h_val))) {
		list_remove(T, index);
		continue;
	}
	t1 = list_get(T, index);
	for (int j = 0; j < HASH_FUNS; j++) {
		if (j != z) {
			auto el = allNodes_table[jenkins_hash(j, t1) % HASH_SIZE];
			if (el != NULL && el == t1 &&
					((list_get(T, index)->g_val + list_get(T, index)->h_val) >= (el->g_val + el->h_val))) {
				list_remove(T, index);
				break;
			}
		}
	}
}

__global__ void push_to_queues(int k, heap **open_list, llist *S, int off) {
	for (int i = threadIdx.x; i < S->length; i += blockDim.x) {
		AStarNode *t1 = list_get(S, i);
		if (t1 != NULL) {
			pushNode(open_list[(i + off) % k], t1);
			open_list.increase(t1->open_handle); 
			atomicAdd(&processed, 1);
			atomicAdd(&total_open_list_size, 1);
		}
		__syncthreads();
	}
}

__device__ getNeighbors(int curr, int* neighbors, int* n_size, Instance* inst) {
	int candidates[4] = {curr + 1, curr - 1, curr + inst->num_of_cols, curr - inst->num_of_cols};
	*n_size = 0;
	for (int i=0; i<4; i++)
	{
		int next = candidates[i];
		if (next >= 0 && next < inst->map_size && (!inst->my_map[next])) {
			neighbors[count] = next;
			*n_size ++;
		}
	}
	return;
} 