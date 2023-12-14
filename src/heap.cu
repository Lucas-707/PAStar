#include "heap.h"
#include "GAStar.h"
#include <stdlib.h>
#include <stdio.h>

__device__ static void swap(GNode **s1, GNode **s2);

heap **heaps_create(int k) {
	heap **Q_cpu = (heap**)malloc(k * sizeof(heap*));
	heap **Q_dev = NULL;
	for (int i = 0; i < k; i++) {
		Q_cpu[i] = heap_create(16 * 1024);
	}
	cudaMalloc(&Q_dev, k * sizeof(heap*));
	cudaMemcpy(Q_dev, Q_cpu, k * sizeof(heap*), cudaMemcpyDefault);
	free(Q_cpu);
	return Q_dev;
}

heap *heap_create(int capacity) {
	heap heap_cpu;
	heap *heap_dev;
	heap_cpu.size = 0;
	cudaMalloc(&(heap_cpu.nodes), (capacity + 1) * sizeof(GNode*));
	cudaMemset(heap_cpu.nodes, 0, (capacity + 1) * sizeof(GNode*));

	cudaMalloc(&heap_dev, sizeof(heap));
	cudaMemcpy(heap_dev, &heap_cpu, sizeof(heap), cudaMemcpyDefault);
	return heap_dev;
}

void heaps_destroy(heap **Q_dev, int k) {
	heap **Q_cpu = (heap**)malloc(k * sizeof(heap*));
	cudaMemcpy(Q_cpu, Q_dev, k * sizeof(heap*), cudaMemcpyDefault);
	for (int i = 0; i < k; i++) {
		heap_destroy(Q_cpu[i]);
	}
	free(Q_cpu);
	cudaFree(Q_dev);
}

void heap_destroy(heap *heap_dev) {
	heap heap_cpu;
	cudaMemcpy(&heap_cpu, heap_dev, sizeof(heap), cudaMemcpyDefault);
	cudaFree(heap_cpu.nodes);
	cudaFree(heap_dev);
}

__device__ void heap_insert(heap *heap, GNode *node) {
	heap->size++;
	heap->nodes[heap->size] = node;
	int current = heap->size;
	while (current > 1 && (heap->nodes[current]->g_val + heap->nodes[current]->h_val) < (heap->nodes[current / 2]->g_val + heap->nodes[current / 2]->h_val)) {
		swap(&(heap->nodes[current]), &(heap->nodes[current / 2]));
		current /= 2;
	}
}

__device__ GNode *heap_extract(heap *heap) {
	GNode *res = heap->nodes[1];
	heap->nodes[1] = heap->nodes[heap->size];
	heap->nodes[heap->size] = NULL;
	heap->size--;
	int current = 1;
	while (current < heap->size) {
		int smallest = current;
		int child = 2 * current;
		if (child <= heap->size && (heap->nodes[child]->g_val + heap->nodes[child]->h_val) < (heap->nodes[smallest]->g_val + heap->nodes[smallest]->h_val)) {
			smallest = child;
		}
		child = 2 * current + 1;
		if (child <= heap->size && (heap->nodes[child]->g_val + heap->nodes[child]->h_val) < (heap->nodes[smallest]->g_val + heap->nodes[smallest]->h_val)) {
			smallest = child;
		}
		if (smallest == current) {
			break;
		}
		swap(&(heap->nodes[current]), &(heap->nodes[smallest]));
		current = smallest;
	}
	return res;
}

__device__ bool heaps_empty(heap **heaps, int k) {
	for (int i = 0; i < k; i++) {
		if (heaps[i]->size != 0) return false;
	}
	return true;
}

__device__ int heaps_min(heap **heaps, int k) {
	int best_f = INT_MAX;
	for (int i = 0; i < k; i++) {
		GNode *current_best = heaps[i]->nodes[1];
		if (current_best != NULL && (current_best->g_val + current_best->h_val) < best_f) {
			best_f = (current_best->g_val + current_best->h_val);
		}
	}
	return best_f;
}


__device__ static void swap(GNode **s1,  GNode **s2) {
	GNode *tmp = *s1;
	*s1 = *s2;
	*s2 = tmp;
}