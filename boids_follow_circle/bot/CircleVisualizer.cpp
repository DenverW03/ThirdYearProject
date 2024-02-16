#include <stage.hh>

using namespace Stg;
using namespace CircleVIS;

// Constructor for the class
CircleVisualizer::CircleVisualizer():Visualizer("circle", "circlevis"){
  this->cdata.x = 0;
  this->cdata.y = 0;
  this->cdata.radians = 0;
}

// Draw a vertex
inline void CircleVisualizer::drawPoint(double x, double y){
  glVertex2f(x, y);
}

void CircleVisualizer::Visualize( Model* mod, Camera* cam ){
    glBegin();

    glColor3f(1,0,0);
    drawPoint(0, 0);
    this->cdata.x = 1;
    this->cdata.y = 1;
    drawPoint(this->cdata.x, this->cdata.y);

    glEnd();
}