#ifndef _IMAGELOADER_H_
#define _IMAGELOADER_H_

#include "openGL-headers.h"
#include <string>

// Texture
extern GLuint texHandle;

struct BitMapFile
{
  int sizeX;
  int sizeY;
  unsigned char *data;
};

BitMapFile *getBMPData(std::string filename);
void loadTextures(std::string filename);

#endif
