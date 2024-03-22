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
    #define avoidanceFactor 0.1
    #define alignmentFactor 0.05

    // Obstruction avoidance parameters, the distance is the same as the range for the sonar
    #define avoidObstructionDistance 3
    #define avoidObstructionFactor 0.1

    // Some definitions for colors
    #define black 0x000000ff
    #define blue 0x0000ffff

    // The number of cameras used up to 16 for current model, vary number for testing
    #define camCount 16

    // Defining structs for passing converted speed values
    typedef struct {
        double linearVel;
        double rotationalVel;
    } NHVelocities;

    typedef struct {
        double xvel;
        double yvel;
    } HVelocities;

    // Sensor setup data struct
    typedef struct {
        SimpleRobot *robot;
        ModelBlobfinder *bf;
        int num;
    } SensorInputData;

    // Struct that holds the boids necessary data
    typedef struct {
        double separateX;
        double separateY;
        double separateXObs;
        double separateYObs;
        double averageXPos;
        double averageYPos;
        int numNeighbours;
    } BoidData;
    
    public:
        // Variable declarations
        ModelPosition *pos;
        ModelBlobfinder **cameras;
        double xVel;
        double yVel;
        BoidData boidData;

        // Angles of the cameras places on the bot
        double angles[16] = {0, 22.5, 45, 67.5, 90, 112.5, 135, 157.5, 180, -22.5, -45, -67.5, -90, -112.5, -135, -157.5};

        // Function declarations
        SimpleRobot();
        SimpleRobot(ModelPosition *modelPos, Pose pose);
        static int SensorUpdate(Model *, SensorInputData *data);
        static int PositionUpdate(Model *, SimpleRobot *robot);
        static std::pair<double, double> CalculatePosition(double a, Pose pose, double distance);
        static double CalculateDistance(Pose pose, SimpleRobot *robot);
        static HVelocities CalculateHolonomic(double xvel, double turnvel, SimpleRobot *robot);
        static NHVelocities CalculateNonHolonomic(double xvel, double yvel, SimpleRobot *robot);
        Pose GetPose();
    };
}

#endif
