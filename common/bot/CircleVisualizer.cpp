#include <stage.hh>
#include "CircleVisualizer.hh"
#include <cmath>

using namespace Stg;
using namespace CircleVIS;

// Constructor for the class
CircleVIS::CircleVisualizer::CircleVisualizer(double x, double y, double radius):Visualizer("circle", "circlevis"){
  this->cdata.x = x;
  this->cdata.y = y;
  this->cdata.radius = radius;
}

// Draw a vertex
void CircleVIS::CircleVisualizer::drawPoint(double x, double y){
    glVertex2f(x, y);
}

// Callback for drawing the visualisation
void CircleVIS::CircleVisualizer::Visualize(Stg::Model* mod, Stg::Camera* cam) {
    glBegin(GL_LINES);

    glColor3f(0, 0, 0);

    for (int i = 0; i < numPoints; i++) {
        double xc1, yc1, xc2, yc2;
        double angle1 = ((2 * M_PI) / numPoints) * i;
        double angle2 = ((2 * M_PI) / numPoints) * ((i + 1) % numPoints);

        // Generating the coordinate points at the angles
        xc1 = this->cdata.x + (this->cdata.radius * cos(angle1));
        yc1 = this->cdata.y + (this->cdata.radius * sin(angle1));
        xc2 = this->cdata.x + (this->cdata.radius * cos(angle2));
        yc2 = this->cdata.y + (this->cdata.radius * sin(angle2));

        // Drawing line segment between consecutive points
        glVertex2f(xc1, yc1);
        glVertex2f(xc2, yc2);
    }

    glEnd();
}
