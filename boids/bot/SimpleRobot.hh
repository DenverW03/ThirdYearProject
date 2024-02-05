#ifndef SIMPLE_ROBOT
#define SIMPLE_ROBOT
#include <stage.hh>  // Include the appropriate library for Stg

using namespace Stg;

namespace SimpleRBT {
    class SimpleRobot {

    // Some class parameter definitions
    #define visionRange 8
    #define cohesionFactor 0.005
    #define avoidanceDistance 2
    #define avoidanceFactor 0.01
    #define alignmentFactor 0.05

    // Obstruction avoidance parameters, the distance is the same as the range for the sonar
    #define avoidObstructionDistance 3
    #define avoidObstructionFactor 0.1

    // Some definitions for colors
    #define black 0x000000ff
    #define blue 0x0000ffff

    // The number of cameras used
    #define camCount 16

    // Defining a struct for passing converted speed values

    typedef struct{
        double linearVel;
        double rotationalVel;
    } NHVelocities;
    
    public:

        // Variable declarations

        ModelPosition *pos;
        ModelBlobfinder **cameras;
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