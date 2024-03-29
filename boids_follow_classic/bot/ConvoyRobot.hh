#ifndef CONVOY_ROBOT
#define CONVOY_ROBOT
#include <stage.hh>  // Include the appropriate library for Stg
#include "Parameters.hh"

using namespace Stg;

namespace ConvoyRBT {
    class ConvoyRobot {
    // Some definitions for colors
    #define black 0x000000ff
    #define blue 0x0000ffff
    #define red 0xff0000ff

    // The number of cameras used up to 16 for current model, vary number for testing
    #define camCount 16

    // Defining a struct for passing converted speed values
    typedef struct {
        double linearVel;
        double rotationalVel;
    } NHVelocities;

    typedef struct {
        double xvel;
        double yvel;
    } HVelocities;

    // Data structure for transferring sensor input data
    typedef struct {
        ConvoyRobot *robot;
        ModelBlobfinder *bf;
        int num;
    } SensorInputData;

    // Structure of the necessary boids data
    typedef struct {
        double separateX;
        double separateY;
        double separateXObs;
        double separateYObs;
        double averageXPos;
        double averageYPos;
        int numNeighbours;
    } BoidData;

    // Alignment data structure, linked list of length 2 in implementation
    struct VipVelocityNode {
        double xpos;
        double ypos;
        struct VipVelocityNode *second;
    };
    
    public:

        // Variable declarations
        ModelPosition *pos;
        ModelBlobfinder **cameras;
        double xVel;
        double yVel;
        BoidData boidData;

        // These variables are used in the VIP velocity estimation
        struct VipVelocityNode *stack;
        unsigned long lastSysTime;

        // Used for data reporting
        int id;
        unsigned long startTime;

        // Angles of the cameras places on the bot
        double angles[16] = {0, 22.5, 45, 67.5, 90, 112.5, 135, 157.5, 180, -22.5, -45, -67.5, -90, -112.5, -135, -157.5};

        // Storing distances for testing data output
        std::vector<double> testingDistances;

        // Function declarations
        ConvoyRobot();
        ConvoyRobot(ModelPosition *modelPos, Pose pose, int id);
        static int SensorUpdate(Model *, SensorInputData *data);
        static int PositionUpdate(Model *, ConvoyRobot *robot);
        static void TestingStall(ConvoyRobot *robot);
        static std::pair<double, double> CalculatePosition(double a, Pose pose, double distance);
        static double CalculateDistance(Pose pose, ConvoyRobot *robot);
        static void push(double xvel, double yvel, ConvoyRobot *robot);
        static struct VipVelocityNode* newNode(double xpos, double ypos);
        static HVelocities CalculateHolonomic(double xvel, double turnvel, ConvoyRobot *robot);
        static NHVelocities CalculateNonHolonomic(double xvel, double yvel, ConvoyRobot *robot);
        Pose GetPose();
    };
}

#endif
