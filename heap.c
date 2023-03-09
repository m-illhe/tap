#include "heap.h"
#include <stdlib.h>
#include <stdio.h>

heap heap_create(int k, int (*f)(const void *, const void *)){
  heap h = malloc(sizeof(*h));
  h->n = 0;
  h->nmax = k;
  h->array = malloc((h->nmax + 1) * sizeof(void *));
  h->f = f;
  return h;
}

void heap_destroy(heap h) {
 free(h->array);
 free(h);
}

bool heap_empty(heap h) { 
  return (h->n == 0); 
  }

bool heap_add(heap h, void *object) {
  if (!(h->n < h->nmax)){
    //printf("no place");
    return true;
  }
  if (h->n == 0){
    h->array[1] = object;
    h->n += 1;
    return false;
  }
  int index_fils = (h->n) + 1;
  int index_pere = index_fils / 2;
  h->array[index_fils] = object;
  while(h->f(h->array[index_pere], h->array[index_fils]) > 0){
    void* temp = h->array[index_fils];
    h->array[index_fils] = h->array[index_pere];
    h->array[index_pere] = temp ;
    index_fils = index_pere;
    index_pere = index_pere / 2;
    if ( index_pere == 0){
      break;
    }
  }
  h->n += 1;
  return false;
}

void *heap_top(heap h) {
  if (h == NULL){
    printf("tas binaire NULL");
    return NULL;
  }
  return h->array[1];
}

void *heap_pop(heap h) {
  if(h->n == 0) return NULL;
  void *first = h->array[1];
  h->array[1] = h->array[h->n];
  int index_pere = 1;
  int index_fils_gauche = index_pere * 2;
  int index_fils_droit = index_fils_gauche + 1;
  int swap_index = -1;
  
  while(index_fils_gauche < h->n){
    if (index_fils_droit > (h->n)){
      swap_index = index_fils_gauche;
    } else if (h->f(h->array[index_fils_gauche], h->array[index_fils_droit]) < 0) { 
      swap_index = index_fils_gauche;
    } else {
      swap_index = index_fils_droit;
    }

    if (h->f(h->array[index_pere], h->array[swap_index]) > 0){
      void* temp = h->array[swap_index];
      h->array[swap_index] = h->array[index_pere];
      h->array[index_pere] = temp;
    } else{
      break;
    }
    index_pere = swap_index;
    index_fils_gauche = index_pere *2;
    index_fils_droit = index_fils_gauche + 1; 
   
  }

  h->n = (h->n) - 1;

  return first;
}
