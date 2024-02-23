#ifndef CONVOY_ROBOT
#define CONVOY_ROBOT
#include <stage.hh>  // Include the appropriate library for Stg
#include "CircleVisualizer.hh"
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

        typedef struct {
            ConvoyRobot *robot;
            ModelBlobfinder *bf;
            int num;
        } SensorInputData;

        typedef struct {
            double closeDx;
            double closeDy;
            double closeDxObs;
            double closeDyObs;
            double closeDxVip;
            double closeDyVip;
            double averageXPos;
            double averageYPos;
            double averageXVel;
            double averageYVel;
            int numNeighbours;
        } BoidData;

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

            // Angles of the cameras places on the bot

            double angles[16] = {0, 22.5, 45, 67.5, 90, 112.5, 135, 157.5, 180, -22.5, -45, -67.5, -90, -112.5, -135, -157.5};

            // Function declarations

            ConvoyRobot();
            ConvoyRobot(ModelPosition *modelPos, Pose pose);
            static int SensorUpdate(Model *, SensorInputData *data);
            static int PositionUpdate(Model *, ConvoyRobot *robot);
            static std::pair<double, double> CalculatePosition(double a, Pose pose, double distance);
            static double CalculateDistance(Pose pose, ConvoyRobot *robot);
            static std::vector<std::pair<double, double>> GeneratePoints(Pose pose, double radius);
            static void push(double xvel, double yvel, ConvoyRobot *robot);
            static struct VipVelocityNode* newNode(double xpos, double ypos);
            static HVelocities CalculateHolonomic(double xvel, double turnvel, ConvoyRobot *robot);
            static NHVelocities CalculateNonHolonomic(double xvel, double yvel, ConvoyRobot *robot);
            Pose GetPose();
    };
}

#endif
