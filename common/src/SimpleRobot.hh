#ifndef SIMPLE_ROBOT
#define SIMPLE_ROBOT
#include <stage.hh>  // Include the appropriate library for Stg

using namespace Stg;

namespace SimpleRBT {
    class SimpleRobot {

    // Some class parameter definitions

    #define visionRange 5
    #define cohesionFactor 0.01
    #define avoidanceDistance 1
    #define avoidanceFactor 0.5
    #define alignmentFactor 0.05

    // Defining a struct for passing converted speed values

    typedef struct{
        double linearVel;
        double rotationalVel;
    } NHVelocities;
    
    public:

        // Variable declarations

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
        static NHVelocities CalculateNonHolonomic(double xvel, double yvel, SimpleRobot *robot);
    };
}

#endif
