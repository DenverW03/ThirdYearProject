#ifndef SIMPLE_ROBOT
#define SIMPLE_ROBOT
#include <stage.hh>  // Include the appropriate library for Stg

using namespace Stg;

namespace SimpleRBT {
    class SimpleRobot {
    public:
        ModelPosition *pos;
        ModelRanger *laser;
        double xVel;
        double yVel;
        double turnVel;
        SimpleRobot* robots;
        // Function declarations
        SimpleRobot();
        SimpleRobot(ModelPosition *modelPos, int x, int y, SimpleRobot *robots);
        static int SensorUpdate(Model *, SimpleRobot *robot);
        static int PositionUpdate(Model *, SimpleRobot *robot);
        Pose GetPose();
        static double CalculateDistance(Pose pose, SimpleRobot *robot);
    };
}

#endif
