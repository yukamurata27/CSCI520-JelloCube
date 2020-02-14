/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/


#ifndef _SHOWCUBE_H_
#define _SHOWCUBE_H_

void LoadTexture(const char * filename, int width, int height);

void showCube(struct world * jello);
void showBoundingBox();
void showCornellBox();
void showSmallBox();

#endif
