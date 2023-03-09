#include "tools.h"
#include "tsp_brute_force.h"
#include "tsp_prog_dyn.h"

//
//  TSP - PROGRAMMATION DYNAMIQUE
//
//  -> compléter uniquement tsp_prog_dyn()
//  -> la structure "cell" est définie dans "tsp_prog_dyn.h"
//

// Renvoie l'ensemble S\{i}.
int DeleteSet(int S, int i) { return S & ~(1 << i); }

int ExtractPath(cell **D, int t, int S, int n, int *Q) {
  /*
    Construit dans Q le chemin entre les points V[n-1] et V[t] passant
    par tous les points de S, c'est-à-dire le chemin extrait depuis la
    case D[t][S]. La longueur du chemin Q ainsi extrait vaut en
    principe D[t][S].length. Renvoie le nombre de points du chemin
    Q. On supposera Q[] assez grand pour revoir tous ses points.
  */
  if(D[0][1].length < 0) return 0; // si D n'a pas été remplie

  // phase 1: extrait Q = V[t] ... V[n-1] dans cet ordre

  Q[0] = t;                   // écrit le dernier point V[t] en premier
  int k = 1;                  // k = taille de Q = nombre de points écrits dans Q
  while (Q[k - 1] != n - 1) { // on s'arrête lorsque le point V[n-1] est atteint
    Q[k] = D[Q[k - 1]][S].pred;
    S = DeleteSet(S, Q[k - 1]);
    k++;
  }

  // phase 2: renverse Q pour avoir Q = V[n-1] ... V[t], au cas où la
  // fonction dist() ne soit pas symétrique

  int p=0, q=k-1;
  while(p<q){
    SWAP(Q[p], Q[q], t);
    p++; q--;
  }

  return k;
}

double tsp_prog_dyn(point *V, int n, int *Q) {
  /*
    Version programmation dynamique du TSP. La tournée optimale
    calculée doit être écrite dans la permutation Q, tableau qui doit
    être alloué par l'appelant. La fonction doit renvoyer aussi la
    valeur de la tournée Q ou 0 s'il y a eut un problème, comme la
    pression de 'q' pour sortir de l'affichage.
    
    La table D est un tableau 2D de "cell" indexé par t ("int"),
    l'indice d'un point V[t], et S ("int") représentant un ensemble
    d'indices de points.

    o D[t][S].length = longueur minimum d'un chemin allant de V[n-1] à
      V[t] qui visite tous les points d'indice dans S

    o D[t][S].pred = l'indice du point précédant V[t] dans le chemin
      ci-dessus de longueur D[t][S].length

    NB1: Ne pas lancer tsp_prog_dyn() avec n>31 car:
         o les entiers (int sur 32 bits) ne seront pas assez grands
           pour représenter tous les sous-ensembles;
         o pour n=32, il faudra environ n*2^n / 10^9 = 137 secondes sur
           un ordinateur à 1 GHz, ce qui est un peu long.
         o l'espace mémoire, le malloc() pour la table D, risque d'être
           problématique: 32*2^32*sizeof(cell) représente déjà 1536 Go
           de mémoire.
         En pratique on peut monter facilement jusqu'à n=24 pour une
         dizaine de secondes de calcul.
 
    NB2: La variable globale "running" indique si l'affichage
         graphique est actif, la pression de 'q' la faisant passer à
         faux. L'usage de "running" permet à l'utilisateur de sortir
         des boucles de calcul lorsqu'elles sont trop longues. Donc
         dans la phase de test, pensez à ajouter "&& running" dans vos
         conditions de boucle pour quitter en court de route si c'est
         trop long.
  */

  //-------------------------------------------------------------
  // Phase 1: Déclaration de la table.
  //
  // Elle comporte (n-1)*2^(n-1) "cell". NB: la colonne S=0
  // (l'ensemble vide) n'est pas utilisée.

  int const L = n-1;    // L = nombre de lignes = indice du dernier point
  int const C = 1 << L; // C = nombre de colonnes

  cell **D = malloc(L*sizeof(cell*)); // L=n-1 lignes
  for (int t=0; t<L; t++) D[t] = malloc(C*sizeof(cell)); // C=2^{n-1} colonnes
  D[0][1].length=-1; // pour savoir si la table a été remplie

  //-------------------------------------------------------------
  // Phase 2: Remplissage de la table.
  //
  // o Pour toutes les colonnes S faire ...
  //   o Pour chaque ligne t de D[][S] faire ...

  // Rappel de la formule pour remplir la table D:
  // si card(S)=1, alors D[t][S] = d(V[n-1], V[t]) avec S={t};
  // si card(S)>1, alors D[t][S] = min_x { D[x][S\{t}] + d(V[x], V[t]) }
  // avec t∈S et x∈S\{t}. NB: pour calculer T = S\{t}, poser
  // T=DeleteSet(S,t). On peut tester si t∈S aussi avec
  // DeleteSet(S,t).
  for (int S = 1; S < C; S++){
      for ( int t = 0; t < L; t++){
        int T = DeleteSet(S, t);
        if(T == S){ // On vérifie si t est inclus dans S cf ligne 108,109
          continue;
        }
        else if (T == 0){ //on est dans le cas card(S) = 1
          D[t][S].length = dist(V[t], V[n-1]);
          D[t][S].pred   = n-1;
        }
        else { // t est dans S et S a au moins 2 elements 
          double distance = DBL_MAX;
          int pred = -1;
          double ndist;
          for( int x = 1; x < L; x++){
            if(DeleteSet(T, x) == T){ continue;}
            ndist = D[x][T].length + dist(V[x], V[t]);
            if (ndist < distance){
              distance = ndist;
              pred = x;
            }
          }
          D[t][S].length = distance;
          D[t][S].pred = pred;
          //int k = ExtractPath(D, t, S, n, Q); // extrait Q depuis D[t][S]
          //drawPath(V, n, Q, k);               // dessine le chemin Q
          //if (!running) return 0;
        }
      }    
    }

  // Toujours dans la boucle, mais à la fin, lorsque le calcul de la
  // cellule D[t][S] est fait (c'est-à-dire les valeurs D[t][S].length
  // et D[t][S].pred correctement calcuées), vous pouvez afficher le
  // chemin Q correspondant à la case D[t][S] à l'aide d'ExtractPath()
  // puis de drawPath() comme ci-dessous:
  //
  // int k = ExtractPath(D, t, S, n, Q); // extrait Q depuis D[t][S]
  // drawPath(V, n, Q, k);               // dessine le chemin Q
  // if (!running) return 0;             // on arrête tout si 'q' est pressée
  //
  // Ici, c'est la fin des deux boucles. Le calcul de la table est
  // terminé.

  //-------------------------------------------------------------
  // Phase 3: Extraction de la tournée optimale.
  //
  // On notera w la longueur de la tournée optimale qui reste à
  // calculer. NB: si le calcul a été interrompu (pression de 'q'), il
  // faut renvoyer 0.

  double w = 0; // valeur par défaut

 if (running) {
    // Si 'q' n'a pas été pressée, il faut calculer w puis extraire la
    // tournée Q correspondante à l'aide d'ExtractPath(...,Q).
    int k = ExtractPath(D,0,C-1,n,Q);

    for(int i = 0; i < k; i++){
      w +=  dist(V[Q[i]], V[Q[i+1]]);
    }
    w += dist(V[Q[n-1]], V[Q[0]]);
  }


  //-------------------------------------------------------------
  // Phase 4: Valeur retour en libérant la table D.
  for (int t=0; t<L; t++) free(D[t]);
  free(D);

  return w;
}

