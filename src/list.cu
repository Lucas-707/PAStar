#include "list.h"
#include "GAStar.h"
#include <assert.h>
#include <stdio.h>

llist **lists_create(int lists, int capacity) {
	llist **lists_cpu = (llist**)malloc(lists * sizeof(llist*));
	llist **lists_gpu = NULL;
	for (int i = 0; i < lists; i++) {
		lists_cpu[i] = list_create(capacity);
	}
	HANDLE_RESULT(cudaMalloc(&lists_gpu, lists * sizeof(llist*)));
	HANDLE_RESULT(cudaMemcpy(lists_gpu, lists_cpu, lists * sizeof(llist*), cudaMemcpyDefault));
	free(lists_cpu);
	return lists_gpu;
}

llist *list_create(int capacity) {
	llist list_cpu;
	llist *list_gpu;
	list_cpu.length = 0;
	list_cpu.capacity = capacity;
	HANDLE_RESULT(cudaMalloc(&(list_cpu.arr), (capacity + 1) * sizeof(state*)));
	HANDLE_RESULT(cudaMalloc(&list_gpu, sizeof(struct list)));
	HANDLE_RESULT(cudaMemcpy(list_gpu, &list_cpu, sizeof(struct list),
				cudaMemcpyDefault));
	return list_gpu;
}

void lists_destroy(llist **lists_gpu, int lists) {
	llist **lists_cpu = (llist**)malloc(lists * sizeof(llist*));
	HANDLE_RESULT(cudaMemcpy(lists_cpu, lists_gpu, lists * sizeof(llist*), cudaMemcpyDefault));
	for (int i = 0; i < lists; i++) {
		list_destroy(lists_cpu[i]);
	}
	HANDLE_RESULT(cudaFree(lists_gpu));
	free(lists_cpu);
}

void list_destroy(llist *list_gpu) {
	llist list_cpu;
	HANDLE_RESULT(cudaMemcpy(&list_cpu, list_gpu, sizeof(struct llist),
				cudaMemcpyDefault));
	HANDLE_RESULT(cudaFree(list_cpu.arr));
	HANDLE_RESULT(cudaFree(list_gpu));
}
__device__ void list_clear(llist *list) {
	list->length = 0;
}

__device__ void list_insert(llist *list, AStartNode *node) {
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
