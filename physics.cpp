/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "physics.h"

#include <iostream>
using namespace std;

void SpringSystemHelper(struct world * jello, struct point indexA, struct point indexB, struct point &f)
{
  int Ax = indexA.x, Ay = indexA.y, Az = indexA.z;
  int Bx = indexB.x, By = indexB.y, Bz = indexB.z;
  double Llen, diff, dotProdRes, R=1./7;
  point A, B, L, FTmp, vecDiff;

  pCPY(jello->p[Ax][Ay][Az], A);
  pCPY(jello->p[Bx][By][Bz], B);
  pDIFFERENCE(A, B, L);
  pLENGTH(L, Llen);

  // Hook's law
  diff = Llen - R;
  pMULTIPLY(L, -jello->kElastic*diff/Llen, FTmp);
  pSUM(f, FTmp, f);

  // Damping force
  pDIFFERENCE(jello->v[Ax][Ay][Az], jello->v[Bx][By][Bz], vecDiff); // vecDiff = vA - vB
  pDOTPRODUCT(vecDiff, L, dotProdRes);
  pMULTIPLY(L, -jello->dElastic*dotProdRes/Llen/Llen, FTmp);
  pSUM(f, FTmp, f);
}

void SpringSystem(struct world * jello, struct point f[8][8][8])
{
  int i, j, k;
  point indexA, indexB;

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
        // Initialize
        pINIT(f[i][j][k]);
        indexA.x = i;
        indexA.y = j;
        indexA.z = k;

        // Structural springs (6 connections)
        if (i != 0) {
          indexB.x = i-1;
          indexB.y = j;
          indexB.z = k;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        if (i != 7) {
          indexB.x = i+1;
          indexB.y = j;
          indexB.z = k;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        if (j != 0) {
          indexB.x = i;
          indexB.y = j-1;
          indexB.z = k;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        if (j != 7) {
          indexB.x = i;
          indexB.y = j+1;
          indexB.z = k;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        if (k != 0) {
          indexB.x = i;
          indexB.y = j;
          indexB.z = k-1;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        if (k != 7) {
          indexB.x = i;
          indexB.y = j;
          indexB.z = k+1;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }

        //cout << "Result: " << f[i][j][k].x << ", " << f[i][j][k].y << ", " << f[i][j][k].z << endl;

        // Shear springs (20 connections)
        // On x = 0 plane
        if (j != 0 && k != 0) {
          indexB.x = i;
          indexB.y = j-1;
          indexB.z = k-1;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        if (j != 0 && k != 7) {
          indexB.x = i;
          indexB.y = j-1;
          indexB.z = k+1;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        if (j != 7 && k != 0) {
          indexB.x = i;
          indexB.y = j+1;
          indexB.z = k-1;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        if (j != 7 && k != 7) {
          indexB.x = i;
          indexB.y = j+1;
          indexB.z = k+1;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        // On y = 0 plane
        if (i != 0 && k != 0) {
          indexB.x = i-1;
          indexB.y = j;
          indexB.z = k-1;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        if (i != 0 && k != 7) {
          indexB.x = i-1;
          indexB.y = j;
          indexB.z = k+1;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        if (i != 7 && k != 0) {
          indexB.x = i+1;
          indexB.y = j;
          indexB.z = k-1;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        if (i != 7 && k != 7) {
          indexB.x = i+1;
          indexB.y = j;
          indexB.z = k+1;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        // On z = 0 plane
        if (i != 0 && j != 0) {
          indexB.x = i-1;
          indexB.y = j-1;
          indexB.z = k;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        if (i != 0 && j != 7) {
          indexB.x = i-1;
          indexB.y = j+1;
          indexB.z = k;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        if (i != 7 && j != 0) {
          indexB.x = i+1;
          indexB.y = j-1;
          indexB.z = k;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        if (i != 7 && j != 7) {
          indexB.x = i+1;
          indexB.y = j+1;
          indexB.z = k;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        // Diagonal
        if (i != 0 && j != 0 && k != 0) {
          indexB.x = i-1;
          indexB.y = j-1;
          indexB.z = k-1;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        if (i != 0 && j != 0 && k != 7) {
          indexB.x = i-1;
          indexB.y = j-1;
          indexB.z = k+1;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        if (i != 0 && j != 7 && k != 0) {
          indexB.x = i-1;
          indexB.y = j+1;
          indexB.z = k-1;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        if (i != 0 && j != 7 && k != 7) {
          indexB.x = i-1;
          indexB.y = j+1;
          indexB.z = k+1;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        if (i != 7 && j != 0 && k != 0) {
          indexB.x = i+1;
          indexB.y = j-1;
          indexB.z = k-1;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        if (i != 7 && j != 0 && k != 7) {
          indexB.x = i+1;
          indexB.y = j-1;
          indexB.z = k+1;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        if (i != 7 && j != 7 && k != 0) {
          indexB.x = i+1;
          indexB.y = j+1;
          indexB.z = k-1;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        if (i != 7 && j != 7 && k != 7) {
          indexB.x = i+1;
          indexB.y = j+1;
          indexB.z = k+1;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }

        //cout << "Result: " << f[i][j][k].x << ", " << f[i][j][k].y << ", " << f[i][j][k].z << endl;

        // Bend springs (6 connections)
        if (i > 1) {
          indexB.x = i-2;
          indexB.y = j;
          indexB.z = k;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        if (i < 6) {
          indexB.x = i+2;
          indexB.y = j;
          indexB.z = k;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        if (j > 1) {
          indexB.x = i;
          indexB.y = j-2;
          indexB.z = k;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        if (j < 6) {
          indexB.x = i;
          indexB.y = j+2;
          indexB.z = k;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        if (k > 1) {
          indexB.x = i;
          indexB.y = j;
          indexB.z = k-2;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }
        if (k < 6) {
          indexB.x = i;
          indexB.y = j;
          indexB.z = k+2;
          SpringSystemHelper(jello, indexA, indexB, f[i][j][k]);
        }

        //cout << "Result: " << f[i][j][k].x << ", " << f[i][j][k].y << ", " << f[i][j][k].z << endl;
      }
}

void CollisionDetectionHelper(struct world * jello, struct point p, struct point &f)
{
  //int Ax = indexA.x, Ay = indexA.y, Az = indexA.z;
  //int Bx = indexB.x, By = indexB.y, Bz = indexB.z;
  double Llen, diff, dotProdRes, R=0.;
  point A, B, L, FTmp, vecDiff;

  //pCPY(jello->p[Ax][Ay][Az], A);
  //pCPY(jello->p[Bx][By][Bz], B);
  //pDIFFERENCE(A, B, L);

  /*
  pLENGTH(L, Llen);

  // Hook's law
  diff = Llen - R;
  pMULTIPLY(L, -jello->kCollision*diff/Llen, FTmp);
  pSUM(f, FTmp, f);

  // Damping force
  point vec0(0, 0, 0);
  pDIFFERENCE(jello->v[Ax][Ay][Az], vec0, vecDiff); // vecDiff = vA - vB
  pDOTPRODUCT(vecDiff, L, dotProdRes);
  pMULTIPLY(L, -jello->dCollision*dotProdRes/Llen/Llen, FTmp);
  pSUM(f, FTmp, f);
  */
}

bool isOutOfBox(struct point p)
{
  return p.x < -4 || p.y < -4 || p.z < -4 || 4 < p.x || 4 < p.y || 4 < p.z;
}

void CollisionDetection(struct world * jello, struct point f[8][8][8])
{
  int i, j, k;
  point indexA, indexB;

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
        // Initialize
        pINIT(f[i][j][k]);

        if (isOutOfBox(jello->p[i][j][k]))
        {
          return;
        }
      }
}

/* Computes acceleration to every control point of the jello cube, 
   which is in state given by 'jello'.
   Returns result in array 'a'. */
void computeAcceleration(struct world * jello, struct point a[8][8][8])
{
  int i, j, k;
  point FAll[8][8][8], FSpring[8][8][8], FPenalty[8][8][8], FForceField[8][8][8];
  SpringSystem(jello, FSpring);
  CollisionDetection(jello, FPenalty);

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
        pSUM(FAll[i][j][k], FSpring[i][j][k], FAll[i][j][k]);
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
