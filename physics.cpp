/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "physics.h"

#include <iostream>
using namespace std;

/*
void kx(struct world * jello, struct point A, struct point B, struct point flocal)
{
  double Llen, diff, R=1/7;
  point L; // L = B->A

  pDIFFERENCE(A, B, L);
  pLENGTH(L, Llen);
  diff = Llen - R;
  pMULTIPLY(L, -jello->kElastic*diff/Llen, flocal);
}
*/

void Hook(struct world * jello, struct point f[8][8][8])
{
  int i, j, k;
  double Llen, diff, R=1/7;
  point A, B, L, FTmp;

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
        // Initialize
        pINIT(f[i][j][k]);
        pCPY(jello->p[i][j][k], A);

        // Structural springs (6 connections)
        if (i != 0) {
          pCPY(jello->p[i-1][j][k], B);
          //kx(jello, A, B, FTmp);
          //pSUM(f[i][j][k], FTmp, f[i][j][k]);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        if (i != 7) {
          pCPY(jello->p[i+1][j][k], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        if (j != 0) {
          pCPY(jello->p[i][j-1][k], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        if (j != 7) {
          pCPY(jello->p[i][j+1][k], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        if (k != 0) {
          pCPY(jello->p[i][j][k-1], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        if (k != 7) {
          pCPY(jello->p[i][j][k+1], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }

        //cout << "Result: " << f[i][j][k].x << ", " << f[i][j][k].y << ", " << f[i][j][k].z << endl;

        // Shear springs (20 connections)
        ////////// On x = 0 plane //////////
        if (j != 0 && k != 0) {
          pCPY(jello->p[i][j-1][k-1], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        if (j != 0 && k != 7) {
          pCPY(jello->p[i][j-1][k+1], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        if (j != 7 && k != 0) {
          pCPY(jello->p[i][j+1][k-1], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        if (j != 7 && k != 7) {
          pCPY(jello->p[i][j+1][k+1], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        ////////// On y = 0 plane //////////
        if (i != 0 && k != 0) {
          pCPY(jello->p[i-1][j][k-1], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        if (i != 0 && k != 7) {
          pCPY(jello->p[i-1][j][k+1], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        if (i != 7 && k != 0) {
          pCPY(jello->p[i+1][j][k-1], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        if (i != 7 && k != 7) {
          pCPY(jello->p[i+1][j][k+1], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        ////////// On z = 0 plane //////////
        if (i != 0 && j != 0) {
          pCPY(jello->p[i-1][j-1][k], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        if (i != 0 && j != 7) {
          pCPY(jello->p[i-1][j+1][k], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        if (i != 7 && j != 0) {
          pCPY(jello->p[i+1][j-1][k], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        if (i != 7 && j != 7) {
          pCPY(jello->p[i+1][j+1][k], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        ////////// Diagonal //////////
        if (i != 0 && j != 0 && k != 0) {
          pCPY(jello->p[i-1][j-1][k-1], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        if (i != 0 && j != 0 && k != 7) {
          pCPY(jello->p[i-1][j-1][k+1], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        if (i != 0 && j != 7 && k != 0) {
          pCPY(jello->p[i-1][j+1][k-1], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        if (i != 0 && j != 7 && k != 7) {
          pCPY(jello->p[i-1][j+1][k+1], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        if (i != 7 && j != 0 && k != 0) {
          pCPY(jello->p[i+1][j-1][k-1], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        if (i != 7 && j != 0 && k != 7) {
          pCPY(jello->p[i+1][j-1][k+1], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        if (i != 7 && j != 7 && k != 0) {
          pCPY(jello->p[i+1][j+1][k-1], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        if (i != 7 && j != 7 && k != 7) {
          pCPY(jello->p[i+1][j+1][k+1], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }

        //cout << "Result: " << f[i][j][k].x << ", " << f[i][j][k].y << ", " << f[i][j][k].z << endl;

        // Bend springs (6 connections)
        if (i > 1) {
          pCPY(jello->p[i-2][j][k], B);
          //kx(jello, A, B, FTmp);
          //pSUM(f[i][j][k], FTmp, f[i][j][k]);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        if (i < 6) {
          pCPY(jello->p[i+2][j][k], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        if (j > 1) {
          pCPY(jello->p[i][j-2][k], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        if (j < 6) {
          pCPY(jello->p[i][j+2][k], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        if (k > 1) {
          pCPY(jello->p[i][j][k-2], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }
        if (k < 6) {
          pCPY(jello->p[i][j][k+2], B);
          pDIFFERENCE(A, B, L);
          pLENGTH(L, Llen);
          diff = Llen - R;
          pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
          pSUM(f[i][j][k], FTmp, f[i][j][k]);
        }

        //cout << "Result: " << f[i][j][k].x << ", " << f[i][j][k].y << ", " << f[i][j][k].z << endl;
      }
}

/* Computes acceleration to every control point of the jello cube, 
   which is in state given by 'jello'.
   Returns result in array 'a'. */
void computeAcceleration(struct world * jello, struct point a[8][8][8])
{
  int i, j, k;
  point FAll[8][8][8], FHook[8][8][8], FDump[8][8][8], FForceField[8][8][8];
  Hook(jello, FHook);


  // Initialize variables
  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
        pINIT(FAll[i][j][k]);
      }

  // Sum up each force
  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
        pSUM(FAll[i][j][k], FHook[i][j][k], FAll[i][j][k]);
      }

  // a = FAll / m
  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
        pMULTIPLY(FAll[i][j][k], 1/jello->mass, a[i][j][k]);
      }
}

/* performs one step of Euler Integration */
/* as a result, updates the jello structure */
void Euler(struct world * jello)
{
  int i,j,k;
  point a[8][8][8];

  computeAcceleration(jello, a);
  
  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
        jello->p[i][j][k].x += jello->dt * jello->v[i][j][k].x;
        jello->p[i][j][k].y += jello->dt * jello->v[i][j][k].y;
        jello->p[i][j][k].z += jello->dt * jello->v[i][j][k].z;
        jello->v[i][j][k].x += jello->dt * a[i][j][k].x;
        jello->v[i][j][k].y += jello->dt * a[i][j][k].y;
        jello->v[i][j][k].z += jello->dt * a[i][j][k].z;

      }
}

/* performs one step of RK4 Integration */
/* as a result, updates the jello structure */
void RK4(struct world * jello)
{
  point F1p[8][8][8], F1v[8][8][8], 
        F2p[8][8][8], F2v[8][8][8],
        F3p[8][8][8], F3v[8][8][8],
        F4p[8][8][8], F4v[8][8][8];

  point a[8][8][8];


  struct world buffer;

  int i,j,k;

  buffer = *jello; // make a copy of jello

  computeAcceleration(jello, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         pMULTIPLY(jello->v[i][j][k],jello->dt,F1p[i][j][k]);
         pMULTIPLY(a[i][j][k],jello->dt,F1v[i][j][k]);
         pMULTIPLY(F1p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F1v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F2p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F2p[i][j][k]);
         // F2v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F2v[i][j][k]);
         pMULTIPLY(F2p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F2v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F3p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F3v[i][j][k]);
         pMULTIPLY(F3p[i][j][k],1.0,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],1.0,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }
         
  computeAcceleration(&buffer, a);


  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F4p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F4v[i][j][k]);

         pMULTIPLY(F2p[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3p[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1p[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4p[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->p[i][j][k],jello->p[i][j][k]);

         pMULTIPLY(F2v[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4v[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->v[i][j][k],jello->v[i][j][k]);
      }

  return;  
}
