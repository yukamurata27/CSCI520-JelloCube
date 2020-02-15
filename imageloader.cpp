#include "imageloader.h"
#include <fstream>

using namespace std;

GLuint texHandle; // texture

BitMapFile *getBMPData(string filename)
{
	BitMapFile *bmp = new BitMapFile;
	unsigned int size, offset, headerSize;
  
	// Read input file name.
	ifstream infile(filename.c_str(), ios::binary);
 
	// Get the starting point of the image data.
	infile.seekg(10);
	infile.read((char *) &offset, 4); 
   
	// Get the header size of the bitmap.
	infile.read((char *) &headerSize,4);

	// Get width and height values in the bitmap header.
	infile.seekg(18);
	infile.read( (char *) &bmp->sizeX, 4);
	infile.read( (char *) &bmp->sizeY, 4);

	// Allocate buffer for the image.
	size = bmp->sizeX * bmp->sizeY * 24;
	bmp->data = new unsigned char[size];

	// Read bitmap data.
	infile.seekg(offset);
	infile.read((char *) bmp->data , size);
   
	// Reverse color from bgr to rgb.
	int temp;
	for (int i = 0; i < size; i += 3)
	{ 
		temp = bmp->data[i];
		bmp->data[i] = bmp->data[i+2];
		bmp->data[i+2] = temp;
	}

	return bmp;
}

void loadTextures(string filename)
{
	// Local storage for bmp image data.
	BitMapFile *image[1];
   
	// Load the texture.
	image[0] = getBMPData(filename);

	// Activate texture index texture[0]. 
	glBindTexture(GL_TEXTURE_2D, texHandle); 

	// Set texture parameters for wrapping.
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	// Set texture parameters for filtering.
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	// Specify an image as the texture to be bound with the currently active texture index.
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image[0]->sizeX, image[0]->sizeY, 0, 
				GL_RGB, GL_UNSIGNED_BYTE, image[0]->data);
}
