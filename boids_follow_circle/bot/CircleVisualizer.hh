#ifndef CIRCLE_VIS
#define CIRCLE_VIS

#include <stage.hh>

using namespace Stg;

namespace CircleVIS {
    class CircleVisualizer : public Visualizer {

        #define numPoints 720

        // Struct for the circle info
        typedef struct {
            double x;
            double y;
            double radius;
        } CircleData;

        public:
            // The constructor
            CircleVisualizer(double x, double y, double radius);

            // The stage visualise function
            void Visualize( Model* mod, Camera* cam );

            // The struct holding the circle data
            CircleData cdata;

            // Draws a point to the given coordinates
            void drawPoint(double x, double y);
    };
}

#endif