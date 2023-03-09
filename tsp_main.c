#include "tools.h"

//#include "tsp_brute_force.h"
//#include "tsp_prog_dyn.h"
#include "tsp_heuristic.h"
#include "tsp_mst.h"

int main(int argc, char *argv[]) {

  int n = (argc>=2)? atoi(argv[1]) : 10;
  unsigned seed = time(NULL) % 1000;
  printf("seed: %u\n", seed); // pour rejouer la même chose au cas où
  srandom(seed);

  TopChrono(0); // initialise tous les chronos

  point *V = NULL; // tableau de points
  V = generatePoints(n); // n points au hasard
  //V = generateCircles(n,8); // n points sur 8 cercles
  //V = generateConvex(n); // n points aléatoires en position convexe
  //V = generatePower(n,1.4); // n points sur un disque avec loi exponentielle
  //V = generateGrid(&n,3,6); // 18 points sur une grille 3x6
  //V = generateLoad(&n,"points.txt"); // points depuis un fichier

  if(n<=0){
    printf("Error: n=%d, it must be at least one.\n",n);
    exit(1);
  } // ici n>=1

  int *P = malloc(n * sizeof(*P)); // P = la tournée
  P[0] = -1; // permutation qui ne sera pas dessinée par drawTour()

  init_SDL_OpenGL();    // initialisation avant de dessiner
  drawTour(V, n, NULL); // dessine seulement les points

#define MESSAGE					\
  printf("running time: %s\n", TopChrono(1));	\
  printf("waiting for a key ... [h] for help");	\
  fflush(stdout);

#ifdef TSP_BRUTE_FORCE_H
  printf("*** brute-force ***\n");
  running = true; // force l'exécution
  TopChrono(1);   // départ du chrono 1
  printf("value: %g\n", tsp_brute_force(V, n, P));
  MESSAGE;
  update = true;       // force l'affichage
  while (running) {    // affiche le résultat et attend (q pour sortir)
    drawTour(V, n, P); // dessine la tournée
    handleEvent(true); // attend un évènement (=true) ou pas
  }
  printf("\n");

  printf("*** brute-force optimisé ***\n");
  running = true; // force l'exécution
  TopChrono(1);   // départ du chrono 1
  printf("value: %g\n", tsp_brute_force_opt(V, n, P));
  MESSAGE;
  update = true;       // force l'affichage
  while (running) {    // affiche le résultat et attend (q pour sortir)
    drawTour(V, n, P); // dessine la tournée
    handleEvent(true); // attend un évènement (=true) ou pas
  }
  printf("\n");

  /* pour rendre dynamique le calcule */
  /*
  printf("*** brute-force optimisé (affichage dynamique) ***\n");
  running = true; // force l'exécution
  bool redraw = true; // si les points ont changés
  while (running) {   // affiche le résultat et attend (q pour sortir)
    if (redraw){      // recalcule si nécessaire
      TopChrono(1);   // départ du chrono 1
      printf("value: %g\n", tsp_brute_force_opt(V, n, P));
      MESSAGE;
    }
    drawTour(V, n, P); // dessine la tournée
    redraw = handleEvent(true); // attend un évènement (=true) ou pas
  }
  */
#endif

#ifdef TSP_PROG_DYN_H
  printf("*** programmation dynamique ***\n");
  running = true; // force l'exécution
  TopChrono(1);   // départ du chrono 1
  printf("value: %g\n", tsp_prog_dyn(V, n, P));
  MESSAGE;
  update = true;       // force l'affichage
  while (running) {    // affiche le résultat et attend (q pour sortir)
    drawTour(V, n, P); // dessine la tournée
    handleEvent(true); // attend un évènement (=true) ou pas
  }
  printf("\n");
#endif
  
#ifdef TSP_HEURISTIC_H
  printf("*** flip ***\n");
  running = true; // force l'exécution
  TopChrono(1);   // départ du chrono 1
  for(int i =0; i< n; i++){
    P[i] = i;
  }
  printf("value: %g\n", tsp_flip(V, n, P));
  MESSAGE;
  while (running) { // affiche le résultat et attend (q pour sortir)
    update = (first_flip(V, n, P) == 0.0); // force l'affichage si pas de flip possible
    drawTour(V, n, P);   // dessine la tournée
    handleEvent(update); // attend un évènement (si affichage) ou pas
  }
  printf("\n");

  printf("*** greedy ***\n");
  running = true; // force l'exécution
  TopChrono(1);   // départ du chrono 1
  printf("value: %g\n", tsp_greedy(V, n, P));
  MESSAGE;
  bool new_redraw1 = true;
  update = true;    // force l'affichage
  while (running) { // affiche le résultat et attend (q pour sortir)
    if (new_redraw1) tsp_greedy(V, n, P);
    drawTour(V, n, P);   // dessine la tournée
    new_redraw1 = handleEvent(update); // attend un évènement (si affichage) ou pas
    // décommentez la ligne suivante pour avoir greedy+flip
    //update = (first_flip(V, n, P) == 0.0); // force l'affichage si pas de flip
  }
  printf("\n");
#endif

#ifdef TSP_MST_H
  printf("*** mst ***\n");
  running = true; // force l'exécution
  TopChrono(1);   // départ du chrono 1
  graph T=createGraph(n); // graphe vide
  printf("value: %g\n", tsp_mst(V,n,P,T));
  MESSAGE;
  bool new_redraw2 = true;
  update = true; // force l'affichage
  while (running) {
    if (new_redraw2) tsp_mst(V, n, P, T); 
    tsp_flip(V, n, P);
    drawGraph(V, n, P, T);
    new_redraw2 = handleEvent(update); // attend un évènement (si affichage) ou pas
    // décommentez la ligne suivante pour avoir mst+flip
    //update = (first_flip(V, n, P) == 0.0); // force l'affichage si pas de flip
  }
  printf("\n");
  freeGraph(T);
#endif

  // Libération de la mémoire
  TopChrono(-1);
  free(V);
  free(P);
  
  cleaning_SDL_OpenGL();
  return 0;
}