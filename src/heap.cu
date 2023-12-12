#include "heap.h"
#include "GAStar.h"
#include <stdlib.h>
#include <stdio.h>

__device__ static void swap(AStarNode **s1, AStarNode **s2);

heap_open_t **heaps_create(int k) {
	heap_open_t **Q_cpu = (heap_open_t**)malloc(k * sizeof(heap_open_t*));
	heap_open_t **Q_dev = NULL;
	for (int i = 0; i < k; i++) {
		Q_cpu[i] = heap_create(16 * 1024);
	}
	cudaMalloc(&Q_dev, k * sizeof(heap_open_t*));
	cudaMemcpy(Q_dev, Q_cpu, k * sizeof(heap_open_t*), cudaMemcpyDefault);
	free(Q_cpu);
	return Q_dev;
}

heap_open_t *heap_create(int capacity) {
	heap_open_t heap_cpu;
	heap_open_t *heap_dev;
	heap_cpu.size = 0;
	cudaMalloc(&(heap_cpu), (capacity + 1) * sizeof(AStarNode*));
	cudaMemset(heap_cpu, 0, (capacity + 1) * sizeof(AStarNode*));

	cudaMalloc(&heap_dev, sizeof(heap_open_t));
	cudaMemcpy(heap_dev, &heap_cpu, sizeof(heap_open_t), cudaMemcpyDefault);
	return heap_dev;
}

void heaps_destroy(heap_open_t **Q_dev, int k) {
	heap_open_t **Q_cpu = (heap_open_t**)malloc(k * sizeof(heap_open_t*));
	cudaMemcpy(Q_cpu, Q_dev, k * sizeof(heap_open_t*), cudaMemcpyDefault);
	for (int i = 0; i < k; i++) {
		heap_destroy(Q_cpu[i]);
	}
	free(Q_cpu);
	cudaFree(Q_dev);
}

void heap_destroy(heap_open_t *heap_dev) {
	heap heap_cpu;
	cudaMemcpy(&heap_cpu, heap_dev, sizeof(heap_open_t), cudaMemcpyDefault);
	cudaFree(heap_cpu);
	cudaFree(heap_dev);
}

__device__ AStarNode *heap_extract(heap *heap) {
	state *res = heap->states[1];
	heap->states[1] = heap->states[heap->size];
	heap->states[heap->size] = NULL;
	heap->size--;
	int current = 1;
	while (current < heap->size) {
		int smallest = current;
		int child = 2 * current;
		if (child <= heap->size && heap->states[child]->f < heap->states[smallest]->f) {
			smallest = child;
		}
		child = 2 * current + 1;
		if (child <= heap->size && heap->states[child]->f < heap->states[smallest]->f) {
			smallest = child;
		}
		if (smallest == current) {
			break;
		}
		swap(&(heap->states[current]), &(heap->states[smallest]));
		current = smallest;
	}
	return res;
}


__device__ int heaps_min(heap_open_t **heaps, int k) {
	int best_f = INT_MAX;
	for (int i = 0; i < k; i++) {
		AStarNode *current_best = heaps[i].top();
		if (current_best != NULL && current_best->getFVal() < best_f) {
			best_f = current_best->getFVal();
		}
	}
	return best_f;
}


__device__ static void swap(AStarNode **s1,  AStarNode **s2) {
	AStarNode *tmp = *s1;
	*s1 = *s2;
	*s2 = tmp;
}