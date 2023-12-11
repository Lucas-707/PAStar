#ifndef HEAP_H
#define HEAP_H

#include "GAStar.h"


heap_open_t **heaps_create(int k);

heap_open_t *heap_create(int capacity);

void heaps_destroy(heap_open_t **Q_dev, int k);

void heap_destroy(heap_open_t *heap_dev);

__device__ AStarNode *heap_extract(heap_open_t *heap);

__device__ bool heaps_empty(heap_open_t **heaps, int k);

__device__ int heaps_min(heap_open_t **heaps, int k);


#endif //HEAP_H