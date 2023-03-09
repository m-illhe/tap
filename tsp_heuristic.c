#include "tools.h"
#include "tsp_brute_force.h"

//
//  TSP - HEURISTIQUES
//

void reverse(int *T, int p, int q) {
  // Renverse la partie T[p]...T[q] du tableau T avec p<q si
  // T={0,1,2,3,4,5,6} et p=2 et q=5, alors le nouveau tableau T sera
  // {0,1, 5,4,3,2, 6}.
  int tmp;
  for(; p < q; p++, q--){
    SWAP(T[p], T[q], tmp);
  }
}

double first_flip(point *V, int n, int *P) {
  // Renvoie le gain>0 du premier flip réalisable, tout en réalisant
  // le flip, et 0 s'il n'y en a pas.
  for (int i = 0; i < n; i++) {
    for (int j = i + 2; j < n; j++){
      double avant = dist(V[P[i]], V[P[i + 1]]) + dist(V[P[j]], V[P[j + 1]]);
      double apres = dist(V[P[i]], V[P[j]]) + dist(V[P[i + 1]], V[P[j + 1]]);
      if( apres < avant ){
        reverse(P, i + 1, j);
        return avant - apres;
      }
    }
  }
  return 0.0;
} // à initialiser P, par exemple à P[i]=i.

double tsp_flip(point *V, int n, int *P) {
  while(first_flip(V, n, P) != 0){
    drawTour(V, n, P);
  }
  return value(V, n, P);
}

double tsp_greedy(point *V, int n, int *P) {
  // La fonction doit renvoyer la valeur de la tournée obtenue. Pensez
  // à initialiser P, par exemple à P[i]=i.
  ;
  ;
  ;
  return 0.0;
}