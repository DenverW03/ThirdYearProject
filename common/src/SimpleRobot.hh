#ifndef SIMPLE_ROBOT
#define SIMPLE_ROBOT
#include <stage.hh>  // Include the appropriate library for Stg

using namespace Stg;

class SimpleRobot {
public:
    ModelPosition *pos;
    ModelRanger *laser;
    double xVel;
    double yVel;
    double turnVel;
    // Function declarations
    int SensorUpdate(Model *model);
    int PositionUpdate(Model *model);
    SimpleRobot(ModelPosition *modelPos);
};

#endif
