#include <stage.hh>
#include "CircleVisualizer.hh"

using namespace Stg;
using namespace CircleVIS;

// Constructor for the class
CircleVIS::CircleVisualizer::CircleVisualizer():Visualizer("circle", "circlevis"){
  this->cdata.x = 0;
  this->cdata.y = 0;
  this->cdata.radius = 0;
}

// Draw a vertex
void CircleVIS::CircleVisualizer::drawPoint(double x, double y){
    glColor3f(0,0,0);
    glVertex2f(x, y);
}

void CircleVIS::CircleVisualizer::Visualize( Model* mod, Camera* cam ){
    glBegin(GL_POINTS);

    drawPoint(0, 0);
    this->cdata.x = 1;
    this->cdata.y = 1;
    drawPoint(this->cdata.x, this->cdata.y);

    glEnd();
}