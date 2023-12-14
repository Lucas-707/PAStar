#include "GAStar.h"
#include "list.h"
#include "heap.h"

__global__ void clear_open_list(llist *S);
__global__ void fill_open_list(int k, heap **open_list, llist *S, Path path, int goal_location, const Instance *instance, bool *my_map);
__global__ void deduplicate(GNode **H, llist *T);
__global__ void push_to_queues(int k, heap **open_list, llist *S, int off);

__device__ unsigned int jenkins_hash(int j, GNode *node);
__device__ int calculate_index();
__device__ GNode* init_GNode(int loc, int g_val, int h_val, GNode* parent, int timestep, bool in_openlist = false);
__device__ void getNeighbors(int curr, int* neighbors, int* n_size, const Instance *instance, bool *my_map);
__device__ int compute_heuristic(int loc1, int loc2, const Instance *instance);
__device__ int getRowCoordinate(int id, const Instance *instance);
__device__ int getColCoordinate(int id, const Instance *instance);

__device__ int total_open_list_size = 0;
__device__ int found = 0;
__device__ int out_of_memory = 0;

__device__ int processed = 0;
__device__ int steps = 0;
__device__ int heaps_min_before;

Path GAStar::findOptimalPath()
{
    return findSuboptimalPath();
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
	bool* my_map;
	// cudaMalloc((void**)&my_map, (&instance)->my_map.size() * sizeof(bool));
	// cudaMemcpy(my_map, (&instance)->my_map.data(), (&instance)->my_map.size() * sizeof(bool), cudaMemcpyHostToDevice);


	GNode **H;
	cudaMalloc(&H, HASH_SIZE * sizeof(GNode*));
	cudaMemset(H, 0, HASH_SIZE * sizeof(GNode*));
	// priority queues of open lists (Q)
	heap **open_list = heaps_create(k);
	llist *S = list_create(1024 * 1024);
	int total_open_list_size_cpu;
	int found_cpu;
	int out_of_memory_cpu;

	GNode* start = init_GNode(start_location, 0, compute_heuristic(start_location, goal_location), nullptr, 0, 0);

	heap_insert(open_list[0], start);
	atomicAdd(&total_open_list_size, 1);
	int step = 0;

    do {
		clear_open_list<<<1, 1>>>(S);
		cudaDeviceSynchronize();
		fill_open_list<<<BLOCKS, THREADS_PER_BLOCK>>>(k, open_list, S, path, goal_location, &instance, my_map);
		cudaMemcpyFromSymbol(&found_cpu, found, sizeof(int));
		cudaMemcpyFromSymbol(&out_of_memory, found, sizeof(int));
		if (found_cpu) break;
		if (out_of_memory_cpu) break;
		cudaDeviceSynchronize();
		deduplicate<<<BLOCKS, THREADS_PER_BLOCK>>>(H, S);
		cudaDeviceSynchronize();
		push_to_queues<<<1, THREADS_PER_BLOCK>>>(k, open_list, S, step) ;
		cudaDeviceSynchronize();
		cudaMemcpyFromSymbol(&total_open_list_size_cpu, total_open_list_size, sizeof(int));
		step++;
	} while (total_open_list_size_cpu > 0);


	heaps_destroy(open_list, k);
	cudaFree(H);
	cudaDeviceSynchronize();


    // heap_insert(start);
    // H.insert(start);
    // min_f_val = (int) start->getFVal();
    // lower_bound = int(w * min_f_val));

	return path;

}


// inline GNode* GAStar::heap_extract(heap *open_list)
// {
//     auto node = open_list.top(); open_list.pop();
//     // open_list.erase(node->open_handle);
//     node->in_openlist = false;
//     num_expanded++;
//     return node;
// }


// inline void GAStar::heap_insert(heap *open_list, GNode* node)
// {
//     node->open_handle = open_list.push(node);
//     node->in_openlist = true;
//     num_generated++;
// }

// void GAStar::releaseNodes()
// {
// 	// TODO: modify
//     // open_list.clear();
//     for (auto node: H)
//         delete node;
//     H.clear();
// }

__global__ void clear_open_list(llist *S) {
	list_clear(S);
}

__global__ void fill_open_list(int k, heap **open_list, llist *S, Path path, int goal_location, const Instance *instance, bool *my_map) {
	GNode *bestNode = NULL;
	int index = calculate_index();
	if (index == 0) steps++;

	if (open_list[index]->size != 0){
		GNode* curr = heap_extract(open_list[index]);
		atomicSub(&total_open_list_size, 1);
		if (curr->location == goal_location) {
			if (bestNode == NULL || (curr->g_val + curr->h_val) < (bestNode->g_val + bestNode->h_val)) {
				bestNode = curr;
				if (bestNode != NULL && (bestNode->g_val + bestNode->h_val) <= heaps_min(open_list, k)) {
					// Found a better path, update found and return the path
					int found_before = atomicCAS(&found, 0, 1);
					if (found_before == 1) return;
					// updatePath(bestNode, path);
					return;
				}
			}
		}

		// Expand S
		int* neighbors = (int*)malloc(4 * sizeof(int));
		int* n_size;
		getNeighbors(curr->location, neighbors, n_size, instance, my_map);
		for (int j = 0; j < *n_size; j++) {
			int next_location = neighbors[j];
			int next_timestep = curr->timestep + 1;
			int next_g_val = curr->g_val + 1;
			int next_h_val = compute_heuristic(next_location, goal_location, instance);
			GNode *next = init_GNode(next_location, next_g_val, next_h_val, curr, next_timestep);
			list_insert(S, next);
		}
	}
	
}

__global__ void deduplicate(GNode** H, llist *T) {
	int index = calculate_index();
	int z = 0;
	GNode *t1 = list_get(T, index);
	for (int j = 0; j < HASH_FUNS; j++) {
		assert(t1 != NULL);
		auto el = H[jenkins_hash(j, t1) % HASH_SIZE];
		if (el == NULL || t1 == el) {
			z = j;
			break;
		}
	}
	t1 = (GNode*)atomicExch((unsigned long long*)&(H[jenkins_hash(z, t1) % HASH_SIZE]), (unsigned long long)t1);
	if (t1 != NULL && t1 == list_get(T, index) &&
			((list_get(T, index)->g_val + list_get(T, index)->h_val) >= (t1->g_val + t1->h_val))) {
		list_remove(T, index);
		return;
	}
	t1 = list_get(T, index);
	for (int j = 0; j < HASH_FUNS; j++) {
		if (j != z) {
			auto el = H[jenkins_hash(j, t1) % HASH_SIZE];
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
		GNode *t1 = list_get(S, i);
		if (t1 != NULL) {
			heap_insert(open_list[(i + off) % k], t1);
			atomicAdd(&processed, 1);
			atomicAdd(&total_open_list_size, 1);
		}
		__syncthreads();
	}
}

__device__ void getNeighbors(int curr, int* neighbors, int* n_size, const Instance *instance, bool *my_map) {
	int candidates[4] = {curr + 1, curr - 1, curr + instance->num_of_cols, curr - instance->num_of_cols};
	*n_size = 0;
	for (int i=0; i<4; i++)
	{
		int next = candidates[i];
		if (next >= 0 && next < instance->map_size && (!my_map[next])) {
			neighbors[i] = next;
			*n_size ++;
		}
	}
	return;
} 

__device__ unsigned int jenkins_hash(int j, GNode *node) {
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

__device__ GNode* init_GNode(int loc, int g_val, int h_val, GNode* parent, int timestep, bool in_openlist)
{
	GNode* n = (GNode*)malloc(sizeof(GNode));
	n->location = loc;
	n->g_val = g_val;
	n->h_val = h_val;
	n->parent = parent;
	n->timestep = timestep;
	n->in_openlist = in_openlist;

	return n;
}

__device__ int compute_heuristic(int loc1, int loc2, const Instance *instance)
{
	int loc1_x = getRowCoordinate(loc1, instance);
	int loc1_y = getColCoordinate(loc1, instance);
	int loc2_x = getRowCoordinate(loc2, instance);
	int loc2_y = getColCoordinate(loc2, instance);
	return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}

__device__ int calculate_index() {
	return  blockIdx.x * blockDim.x + threadIdx.x;
}

void updatePath(const GNode* goal, vector<PathEntry> &path)
{
    const GNode* curr = goal;
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

__device__ int getRowCoordinate(int id, const Instance *instance) { return id / instance->num_of_cols; }
__device__ int getColCoordinate(int id, const Instance *instance) { return id % instance->num_of_cols; }