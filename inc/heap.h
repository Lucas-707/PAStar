#ifndef HEAP_H
#define HEAP_H

#include "GAStar.h"

struct heap {
	GNode **nodes;
	int size;
};


heap **heaps_create(int k);

heap *heap_create(int capacity);

void heaps_destroy(heap **Q_dev, int k);

void heap_destroy(heap *heap_dev);

__device__ void heap_insert(heap *heap, GNode *node);

__device__ GNode *heap_extract(heap *heap);

__device__ bool heaps_empty(heap **heaps, int k);

__device__ int heaps_min(heap **heaps, int k);


#endif //HEAP_H