#include "list.h"
#include "GAStar.h"
#include <assert.h>
#include <stdio.h>

llist *list_create(int capacity) {
	llist list_cpu;
	llist *list_gpu;
	list_cpu.length = 0;
	list_cpu.capacity = capacity;
	cudaMalloc(&(list_cpu.arr), (capacity + 1) * sizeof(AStarNode*));
	cudaMalloc(&list_gpu, sizeof(struct list));
	cudaMemcpy(list_gpu, &list_cpu, sizeof(struct list),
				cudaMemcpyDefault);
	return list_gpu;
}

void lists_destroy(llist **lists_gpu, int lists) {
	llist **lists_cpu = (llist**)malloc(lists * sizeof(llist*));
	cudaMemcpy(lists_cpu, lists_gpu, lists * sizeof(llist*), cudaMemcpyDefault);
	for (int i = 0; i < lists; i++) {
		list_destroy(lists_cpu[i]);
	}
	cudaFree(lists_gpu);
	free(lists_cpu);
}

void list_destroy(llist *list_gpu) {
	llist list_cpu;
	cudaMemcpy(&list_cpu, list_gpu, sizeof(struct llist),
				cudaMemcpyDefault);
	cudaFree(list_cpu.arr);
	cudaFree(list_gpu);
}
__device__ void list_clear(llist *list) {
	list->length = 0;
}

__device__ void list_insert(llist *list, GNode *node) {
	int index = atomicAdd(&(list->length), 1);
	assert(index < llist->capacity);
	list->arr[index] = node;
}

__device__ void list_remove(llist *list, int index) {
	assert(llist->length < llist->capacity);
	list->arr[index] = NULL;
}

__device__ AStarNode *list_get(llist *list, int index) {
	assert(index < llist->length);
	return list->arr[index];
}
