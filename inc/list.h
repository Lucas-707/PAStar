#ifndef LIST_H
#define LIST_H

#include "GAStar.h"

struct llist {
	int length;
	int capacity;
	AStarNode **arr;
};

llist *list_create(int capacity);

void lists_destroy(llist **lists_gpu, int lists);

void list_destroy(llist *list);

__device__ void list_clear(llist *list);

__device__ void list_insert(llist *list, AStarNode *node);

__device__ void list_remove(llist *list, int index);

__device__ AStarNode *list_get(llist *list, int index);

#endif