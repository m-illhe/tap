#include "tools.h"

//
//  TSP - BRUTE-FORCE
//
// -> la structure "point" est définie dans "tools.h"
// -> tsp_main peut être testé dès les 3 premières fonctions codées
//

double dist(point A, point B) {
  return sqrt((B.x - A.x) * (B.x - A.x) + (B.y - A.y) * (B.y - A.y) );
}

double value(point *V, int n, int *P) {
  double distance = 0;
  for (int i = 0; i < n - 1; i++){
    distance = distance + dist(V[P[i]], V[P[i+1]]);
  }
  distance = distance + dist(V[P[0]], V[P[n-1]]);
  return distance;
}

double tsp_brute_force(point *V, int n, int *Q) {
  int *P = malloc(n*sizeof(*P));
  for (int i = 0; i < n; i++){
    P[i] = i;
  }
  double distance = DBL_MAX;
  
  do{
    double w = value(V, n, P);
    if (w < distance){
      distance = w;
      memcpy(Q, P, n*sizeof(int));
    }
  }
  while (NextPermutation(P, n));
  free(P);
  return distance; 
}

void MaxPermutation(int P, int n, int k) {
  ;
  ;
  ;
  return;
}

double value_opt(point *V, int n, int *P, double w) {
 double distance = 0;
  for (int i = 0; i < n - 1; i++){
    distance = distance + dist(V[P[i]], V[P[i+1]]);
    if (distance > w){
      return w;
    }  
  }
  distance = distance + dist(V[P[0]], V[P[n-1]]);
  if (distance > w){
      return w;
    }
  return distance;
}
double tsp_brute_force_opt(point *V, int n, int *Q) {
  int *P = malloc(n*sizeof(*P));
  for (int i = 0; i < n; i++){
    P[i] = i;
  }
  double distance = DBL_MAX;
  
  do{
    double w = value_opt(V, n, P, distance);
    if (w < distance){
      distance = w;
      memcpy(Q, P, n*sizeof(int));
    }
  }
  while (NextPermutation(P+1, n-1));
  free(P);
  return distance;
  return 0;
}
