/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

  Your name:
  Yuka Murata

*/

#include "jello.h"
#include "showCube.h"
#include "input.h"
#include "physics.h"
#include <iostream>
#include "string.h"
#include "stdio.h"
#include <math.h>

#include <vector>
#include <fstream>

using namespace std;

// camera parameters
double Theta = pi / 6;
double Phi = pi / 6;
double R = 6;

// mouse control
int g_iMenuId;
int g_vMousePos[2];
int g_iLeftMouseButton,g_iMiddleMouseButton,g_iRightMouseButton;

// number of images saved to disk so far
int sprite=0;

// these variables control what is displayed on screen
int shear=0, bend=0, structural=1, pause=0, viewingMode=0, saveScreenToFile=0;

struct world jello;

int windowWidth, windowHeight;

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

ObjLoader myobj;

void myinit()
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(90.0,1.0,0.01,1000.0);

  // set background color to grey
  glClearColor(0.5, 0.5, 0.5, 0.0);

  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);

  glShadeModel(GL_SMOOTH);
  glEnable(GL_POLYGON_SMOOTH);
  glEnable(GL_LINE_SMOOTH);

  // Load and set texture image
  glGenTextures(1, &texHandle);
  loadTextures();

  // Load obj file
  myobj.readfile("obj/bunny.obj");

  return; 
}

void reshape(int w, int h) 
{
  // Prevent a divide by zero, when h is zero.
  // You can't make a window of zero height.
  if(h == 0)
    h = 1;

  glViewport(0, 0, w, h);

  // Reset the coordinate system before modifying
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  // Set the perspective
  double aspectRatio = 1.0 * w / h;
  gluPerspective(60.0f, aspectRatio, 0.01f, 1000.0f);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity(); 

  windowWidth = w;
  windowHeight = h;

  glutPostRedisplay();
}

void display()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // camera parameters are Phi, Theta, R
  gluLookAt(R * cos(Phi) * cos (Theta), R * sin(Phi) * cos (Theta), R * sin (Theta),
	        0.0,0.0,0.0, 0.0,0.0,1.0);


  /* Lighting */
  /* You are encouraged to change lighting parameters or make improvements/modifications
     to the lighting model . 
     This way, you will personalize your assignment and your assignment will stick out. 
  */

  // global ambient light
  GLfloat aGa[] = { 0.0, 0.0, 0.0, 0.0 };
  
  // light 's ambient, diffuse, specular
  GLfloat lKa0[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd0[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat lKs0[] = { 1.0, 1.0, 1.0, 1.0 };

  GLfloat lKa1[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd1[] = { 1.0, 0.0, 0.0, 1.0 };
  GLfloat lKs1[] = { 1.0, 0.0, 0.0, 1.0 };

  GLfloat lKa2[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd2[] = { 1.0, 1.0, 0.0, 1.0 };
  GLfloat lKs2[] = { 1.0, 1.0, 0.0, 1.0 };

  GLfloat lKa3[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd3[] = { 0.0, 1.0, 1.0, 1.0 };
  GLfloat lKs3[] = { 0.0, 1.0, 1.0, 1.0 };

  GLfloat lKa4[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd4[] = { 0.0, 0.0, 1.0, 1.0 };
  GLfloat lKs4[] = { 0.0, 0.0, 1.0, 1.0 };

  GLfloat lKa5[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd5[] = { 1.0, 0.0, 1.0, 1.0 };
  GLfloat lKs5[] = { 1.0, 0.0, 1.0, 1.0 };

  GLfloat lKa6[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd6[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat lKs6[] = { 1.0, 1.0, 1.0, 1.0 };

  GLfloat lKa7[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd7[] = { 0.0, 1.0, 1.0, 1.0 };
  GLfloat lKs7[] = { 0.0, 1.0, 1.0, 1.0 };

  // light positions and directions
  GLfloat lP0[] = { -1.999, -1.999, -1.999, 1.0 };
  GLfloat lP1[] = { 1.999, -1.999, -1.999, 1.0 };
  GLfloat lP2[] = { 1.999, 1.999, -1.999, 1.0 };
  GLfloat lP3[] = { -1.999, 1.999, -1.999, 1.0 };
  GLfloat lP4[] = { -1.999, -1.999, 1.999, 1.0 };
  GLfloat lP5[] = { 1.999, -1.999, 1.999, 1.0 };
  GLfloat lP6[] = { 1.999, 1.999, 1.999, 1.0 };
  GLfloat lP7[] = { -1.999, 1.999, 1.999, 1.0 };
  
  // jelly material color

  GLfloat mKa[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat mKd[] = { 0.3, 0.3, 0.3, 1.0 };
  GLfloat mKs[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat mKe[] = { 0.0, 0.0, 0.0, 1.0 };

  /* set up lighting */
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, aGa);
  glLightModelf(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
  glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);

  // set up cube color
  glMaterialfv(GL_FRONT, GL_AMBIENT, mKa);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, mKd);
  glMaterialfv(GL_FRONT, GL_SPECULAR, mKs);
  glMaterialfv(GL_FRONT, GL_EMISSION, mKe);
  glMaterialf(GL_FRONT, GL_SHININESS, 120);
    
  // macro to set up light i
  #define LIGHTSETUP(i)\
  glLightfv(GL_LIGHT##i, GL_POSITION, lP##i);\
  glLightfv(GL_LIGHT##i, GL_AMBIENT, lKa##i);\
  glLightfv(GL_LIGHT##i, GL_DIFFUSE, lKd##i);\
  glLightfv(GL_LIGHT##i, GL_SPECULAR, lKs##i);\
  glEnable(GL_LIGHT##i)
  
  LIGHTSETUP (0);
  LIGHTSETUP (1);
  LIGHTSETUP (2);
  LIGHTSETUP (3);
  LIGHTSETUP (4);
  LIGHTSETUP (5);
  LIGHTSETUP (6);
  LIGHTSETUP (7);

  // enable lighting
  glEnable(GL_LIGHTING);    
  glEnable(GL_DEPTH_TEST);

  // show the cube
  showCube(&jello);

  glDisable(GL_LIGHTING);

  // show the bounding box
  showBoundingBox();

  // show a small box at the corner
  showSmallBox();

  // show the cornell box environment
  showCornellBox();

  // show the obj that is loaded
  myobj.draw();
 
  glutSwapBuffers();
}

void doIdle()
{
  char s[20]="xxx.ppm";
  int i;
  
  // save screen to file
  s[0] = 48 + (sprite % 1000) / 100;
  s[1] = 48 + (sprite % 100 ) / 10;
  s[2] = 48 + sprite % 10;

  if (saveScreenToFile==1)
  {
    saveScreenshot(windowWidth, windowHeight, s);
    //saveScreenToFile=0; // save only once, change this if you want continuos image generation (i.e. animation)
    sprite++;
  }

  if (sprite >= 300) // allow only 300 snapshots
  {
    exit(0);	
  }

  if (pause == 0)
  {
    // insert code which appropriately performs one step of the cube simulation:
    if (strcmp(jello.integrator, "RK4") == 0) RK4(&jello);
    else if (strcmp(jello.integrator, "Euler") == 0) Euler(&jello);
    else
    {
      printf ("Oops! Your specified integrator doesn't exist!\n");
      exit(0);
    }
  }

  glutPostRedisplay();
}

int main (int argc, char ** argv)
{
  if (argc<2)
  {  
    printf ("Oops! You didn't say the jello world file!\n");
    printf ("Usage: %s [worldfile]\n", argv[0]);
    exit(0);
  }

  readWorld(argv[1],&jello);

  glutInit(&argc,argv);
  
  /* double buffered window, use depth testing, 640x480 */
  glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  
  windowWidth = 640;
  windowHeight = 480;
  glutInitWindowSize (windowWidth, windowHeight);
  glutInitWindowPosition (0,0);
  glutCreateWindow ("Jello cube");

  /* tells glut to use a particular display function to redraw */
  glutDisplayFunc(display);

  /* replace with any animate code */
  glutIdleFunc(doIdle);

  /* callback for mouse drags */
  glutMotionFunc(mouseMotionDrag);

  /* callback for window size changes */
  glutReshapeFunc(reshape);

  /* callback for mouse movement */
  glutPassiveMotionFunc(mouseMotion);

  /* callback for mouse button changes */
  glutMouseFunc(mouseButton);

  /* register for keyboard events */
  glutKeyboardFunc(keyboardFunc);

  /* do initialization */
  myinit();

  /* forever sink in the black hole */
  glutMainLoop();

  return(0);
}

