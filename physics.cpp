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

void CollisionDetectionHelper(struct world * jello, struct point indices, struct point &f)
{
  int px = indices.x, py = indices.y, pz = indices.z;
  //int Bx = indexB.x, By = indexB.y, Bz = indexB.z;
  double Llen, diff, dotProdRes, R=0.;
  point pOut, pSurface, L, FTmp, vecDiff;

  pCPY(jello->p[px][py][pz], pOut);
  pCPY(jello->p[px][py][pz], pSurface);
  if (pSurface.x >= 2.) pSurface.x = 2.;
  else if (pSurface.x <= -2.) pSurface.x = -2.;
  if (pSurface.y > 2.) pSurface.y = 2.;
  else if (pSurface.y <= -2.) pSurface.y = -2.;
  if (pSurface.z >= 2.) pSurface.z = 2.;
  else if (pSurface.z <= -2.) pSurface.z = -2.;
  pDIFFERENCE(pOut, pSurface, L);
  pLENGTH(L, Llen);

  // Hook's law
  diff = Llen - R;
  pMULTIPLY(L, -jello->kCollision*diff/Llen, FTmp);
  pSUM(f, FTmp, f);

  // Damping force
  point vec0 = {0, 0, 0};
  pDIFFERENCE(jello->v[px][py][pz], vec0, vecDiff); // vecDiff = vA - vB
  pDOTPRODUCT(vecDiff, L, dotProdRes);
  pMULTIPLY(L, -jello->dCollision*dotProdRes/Llen/Llen, FTmp);
  pSUM(f, FTmp, f);
}

bool isOutOfBox(struct point p)
{
  return p.x <= -2. || p.y <= -2. || p.z <= -2. || 2. <= p.x || 2. <= p.y || 2. <= p.z;
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
          point indices = {i, j, k};
          CollisionDetectionHelper(jello, indices, f[i][j][k]);
        }
      }
}

void ForceField(struct world * jello, struct point f[8][8][8])
{
  if (jello->resolution == 0) return;
  
  int i, j, k;

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
        // Initialize
        pINIT(f[i][j][k]);
        double interval = jello->resolution/8.; // Or divided by 7????
        double xpos = i/interval, ypos = j/interval, zpos = k/interval;
        int xindices[2] = {floor(xpos), ceil(xpos)};
        int yindices[2] = {floor(ypos), ceil(ypos)};
        int zindices[2] = {floor(zpos), ceil(zpos)};
        double alpha = (xpos-floor(xpos))/interval;
        double beta = (ypos-floor(ypos))/interval;
        double gamma = (zpos-floor(zpos))/interval;
        //cout << "Result: " << alpha << ", " << beta << ", " << gamma << endl;

        point Fxyz, Fx1yz, Fxy1z, Fxyz1, Fx1y1z, Fx1yz1, Fxy1z1, Fx1y1z1;
        pCPY(jello->forceField[xindices[0] * jello->resolution * jello->resolution + yindices[0] * jello->resolution + zindices[0]], Fxyz);
        pCPY(jello->forceField[xindices[1] * jello->resolution * jello->resolution + yindices[0] * jello->resolution + zindices[0]], Fx1yz);
        pCPY(jello->forceField[xindices[0] * jello->resolution * jello->resolution + yindices[1] * jello->resolution + zindices[0]], Fxy1z);
        pCPY(jello->forceField[xindices[0] * jello->resolution * jello->resolution + yindices[0] * jello->resolution + zindices[1]], Fxyz1);
        pCPY(jello->forceField[xindices[1] * jello->resolution * jello->resolution + yindices[1] * jello->resolution + zindices[0]], Fx1y1z);
        pCPY(jello->forceField[xindices[1] * jello->resolution * jello->resolution + yindices[0] * jello->resolution + zindices[1]], Fx1yz1);
        pCPY(jello->forceField[xindices[0] * jello->resolution * jello->resolution + yindices[1] * jello->resolution + zindices[1]], Fxy1z1);
        pCPY(jello->forceField[xindices[1] * jello->resolution * jello->resolution + yindices[1] * jello->resolution + zindices[1]], Fx1y1z1);

        point interpF = {
          alpha*beta*gamma*Fx1y1z1.x + (1-alpha)*beta*gamma*Fxy1z1.x + alpha*(1-beta)*gamma*Fx1yz1.x + alpha*beta*(1-gamma)*Fx1y1z.x + (1-alpha)*(1-beta)*gamma*Fxyz1.x + (1-alpha)*beta*(1-gamma)*Fxy1z.x + alpha*(1-beta)*(1-gamma)*Fx1yz.x + (1-alpha)*(1-beta)*(1-gamma)*Fxyz.x,
          alpha*beta*gamma*Fx1y1z1.y + (1-alpha)*beta*gamma*Fxy1z1.y + alpha*(1-beta)*gamma*Fx1yz1.y + alpha*beta*(1-gamma)*Fx1y1z.y + (1-alpha)*(1-beta)*gamma*Fxyz1.y + (1-alpha)*beta*(1-gamma)*Fxy1z.y + alpha*(1-beta)*(1-gamma)*Fx1yz.y + (1-alpha)*(1-beta)*(1-gamma)*Fxyz.y,
          alpha*beta*gamma*Fx1y1z1.z + (1-alpha)*beta*gamma*Fxy1z1.z + alpha*(1-beta)*gamma*Fx1yz1.z + alpha*beta*(1-gamma)*Fx1y1z.z + (1-alpha)*(1-beta)*gamma*Fxyz1.z + (1-alpha)*beta*(1-gamma)*Fxy1z.z + alpha*(1-beta)*(1-gamma)*Fx1yz.z + (1-alpha)*(1-beta)*(1-gamma)*Fxyz.z
        };
        pSUM(f[i][j][k], interpF, f[i][j][k])
        //cout << "Result: " << f[i][j][k].x << ", " << f[i][j][k].y << ", " << f[i][j][k].z << endl;
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
  ForceField(jello, FForceField);

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
        pSUM(FAll[i][j][k], FPenalty[i][j][k], FAll[i][j][k]);
        pSUM(FAll[i][j][k], FForceField[i][j][k], FAll[i][j][k]);
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
