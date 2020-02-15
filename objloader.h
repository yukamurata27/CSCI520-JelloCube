#ifndef _OBJLOADER_H_
#define _OBJLOADER_H_

#include <vector>
#include <string>
#include <fstream>
#include <math.h>

using namespace std;

#define pi 3.141592653589793238462643383279

class ObjLoader
{
   struct vertex{
        double x;
        double y;
        double z;
   };
   struct face{
         unsigned int v1,v2,v3;
   };
   std::vector<vertex> vetexes;
   std::vector<vertex> vertexnormals;
   std::vector<face> faces;

public:
  void readfile(const char* filename);
  void draw();
  vertex rotateX(const vertex v, const double theta);
  vertex rotateZ(const vertex v, const double theta);
};

extern ObjLoader myobj;

void ObjLoader::readfile(const char *filename) 
{
  string s;
  ifstream fin(filename);
  if(!fin) return;

  while(fin >> s)
  {
    switch(*s.c_str())
    {
      case 'v':
      {
        vertex v;
        fin >> v.x >> v.y >> v.z;
        this->vetexes.push_back(v);
      }
      break;
      /*
      case 'n':
      {
        vertex norm;
        fin >> norm.x >> norm.y >> norm.z;
        this->vertexnormals.push_back(norm);
      } 
      */        
      case 'f':
      {
        face f;
        fin >> f.v1 >> f.v1 >> f.v2 >> f.v2 >> f.v3 >> f.v3;
        this->faces.push_back(f);
      }
      break;
    }
  } 
}

void ObjLoader::draw()
{
  double scaleFactor = 0.5;
  double offset = 1.0;

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glBegin(GL_TRIANGLES);

  //glColor3f(0.99607843137f, 0.49803921568f, 0.61176470588f); // Pink
  glColor4f(0.8,0.8,0.8,0.7);

  vertex res;

  for(int i=0;i<faces.size();i++)
  {
    vertex v1 = vetexes[faces[i].v1-1];
    vertex v2 = vetexes[faces[i].v2-1];
    vertex v3 = vetexes[faces[i].v3-1];

    /*
    vertex n1 = vetexes[faces[i].v1-1];
    vertex n2 = vetexes[faces[i].v2-1];
    vertex n3 = vetexes[faces[i].v3-1];
    */

    //glNormal3f(n1.x+offset, n1.y+offset, n1.z+offset);
    res = rotateX(v1, pi/2);
    res = rotateZ(res, pi/2);
    glVertex3f(
      res.x*scaleFactor-offset,
      res.y*scaleFactor+offset,
      res.z*scaleFactor+offset/2
    );

    res = rotateX(v2, pi/2);
    res = rotateZ(res, pi/2);
    //glNormal3f(n2.x+offset, n2.y+offset, n2.z+offset);
    glVertex3f(
      res.x*scaleFactor-offset,
      res.y*scaleFactor+offset,
      res.z*scaleFactor+offset/2
    );
    
    res = rotateX(v3, pi/2);
    res = rotateZ(res, pi/2);
    //glNormal3f(n3.x+offset, n3.y+offset, n3.z+offset);
    glVertex3f(
      res.x*scaleFactor-offset,
      res.y*scaleFactor+offset,
      res.z*scaleFactor+offset/2
    );
  }
  glEnd();
  glDisable(GL_BLEND);
}

ObjLoader::vertex ObjLoader::rotateX(const vertex v, const double theta)
{
  double sinTheta = sin(theta);
  double cosTheta = cos(theta);
  vertex res = {
    v.x,
    v.y * cosTheta - v.z * sinTheta,
    v.z * cosTheta + v.y * sinTheta
  };
  return res;
}

ObjLoader::vertex ObjLoader::rotateZ(const vertex v, const double theta)
{
  double sinTheta = sin(theta);
  double cosTheta = cos(theta);
  vertex res = {
    v.x * cosTheta - v.y * sinTheta,
    v.y * cosTheta + v.x * sinTheta,
    v.z
  };
  return res;
}

#endif
