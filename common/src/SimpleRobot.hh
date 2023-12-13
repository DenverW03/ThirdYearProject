#ifndef SIMPLE_ROBOT
#define SIMPLE_ROBOT
#include <stage.hh>  // Include the appropriate library for Stg

using namespace Stg;

namespace SimpleRBT {
    class SimpleRobot {
    public:
        ModelPosition *pos;
        ModelRanger *laser;
        double xPos;
        double yPos;
        double xVel;
        double yVel;
        SimpleRobot* robots;
        int numRobots;
        // Function declarations
        SimpleRobot();
        SimpleRobot(ModelPosition *modelPos, Pose pose, SimpleRobot *robots, int numRobots);
        static int SensorUpdate(Model *, SimpleRobot *robot);
        static int PositionUpdate(Model *, SimpleRobot *robot);
        Pose GetPose();
        static double CalculateDistance(Pose pose, SimpleRobot *robot);
    };
}

#endif
