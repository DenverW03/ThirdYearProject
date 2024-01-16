#include "SimpleRobot.hh"
#include <stage.hh>
#include <cmath>
using namespace Stg;
using namespace SimpleRBT;
#define PI 3.14159265358979323846 // defining a value for PI

// Default constructor for init
SimpleRobot::SimpleRobot(){
    // No setup required in implicit constructor
};

// Constructor for the class with appropriate setup
SimpleRobot::SimpleRobot(ModelPosition *modelPos, Pose pose, SimpleRobot *robots, int numRobots) {
    this->pos = nullptr;
    this->laser = nullptr;
    
    // Robot Position Values
    this->xPos = pose.x;
    this->yPos = pose.y;
    
    // Robot Velocity Values
    this->xVel = 1; // This velocity is the useful one for movement in this scenario
    this->yVel = 1; // Even though this bot is non holonomic, treat as holonomic until running bounding algorithm
    
    this->robots = robots; // Holds all the robots in an array
    this->numRobots = numRobots; // Holds the number of robots

    // Setting up positional model and callbacks
    this->pos = modelPos;
    this->pos->AddCallback(Model::CB_UPDATE, model_callback_t(PositionUpdate), this);
    this->laser = (ModelRanger *) (this->pos->GetChild("ranger:0"));
    this->laser->AddCallback(Model::CB_UPDATE, model_callback_t(SensorUpdate), this);
    
    // Subscribing to callback updates
    this->pos->Subscribe();
    this->laser->Subscribe();
    
    // Set the model initial position
    this->pos->SetPose(pose);
}

// Read the ranger data
int SimpleRobot::SensorUpdate(Model *, SimpleRobot* robot) {
    //const std::vector<meters_t> &scan = robot->laser->GetSensors()[0].ranges;
    const std::vector<ModelRanger::Sensor> &sensors = robot->laser->GetSensors();
    // Looping through all sensors
    for(int j=0; j<sensors.size(); j++) {
        const std::vector<meters_t> &scan = robot->laser->GetSensors()[j].ranges;
        uint32_t sampleCount = scan.size();
        if(sampleCount < 1) return 0; // not enough samples is not a legitimate reading for these purposes

        // Check for obstacles in the front
        if (scan[0] < avoidanceDistance) {
            // Calculate the angle to the obstacle for the current sensor
            Pose robotPose = robot->pos->GetPose();
            Pose laserPose = robot->laser->GetSensors()[j].pose;
            double obstacleAngle = laserPose.a - robotPose.a;

            // Update velocities for obstacle avoidance
            robot->xVel += avoidanceFactor * cos(obstacleAngle);
            robot->yVel += avoidanceFactor * sin(obstacleAngle);
        }
    }
    return 0;
}

// Position update function for stage
int SimpleRobot::PositionUpdate(Model *, SimpleRobot* robot) {

    // Separation
    double close_dx = 0;
    double close_dy = 0;

    // Alignment
    double averageXVel = 0;
    double averageYVel = 0;
    double numNeighbours = 0;

    // Cohesion
    double averageXPos = 0;
    double averageYPos = 0;

    // Non-holonmic velocity values
    double linearVel = 0;
    double rotationalVel= 0;
    
    // Looping through all the robots, numRobots given on instantiation of this positional model
    for(int i = 0; i < robot->numRobots; i++) {
        // Exclude self
        if(robot->robots[i].pos == robot->pos) continue;

        // Get the distance of the robot positional model from this robot's positional model
        double distance = CalculateDistance(robot->robots[i].GetPose(), robot);

        // If the distance is within the avoidance range of the robot
        if(distance <= avoidanceDistance) {
            // Separation
            close_dx += robot->xPos - robot->robots[i].xPos;
            close_dy += robot->yPos - robot->robots[i].yPos;
        }

        // If the distance is within the vision range of the robot
        if(distance <= visionRange) {
            // Alignment
            averageXVel += robot->robots[i].xVel;
            averageYVel += robot->robots[i].yVel;
            numNeighbours += 1;

            // Cohesion
            averageXPos += robot->robots[i].xPos;
            averageYPos += robot->robots[i].yPos;
        }
    }

    // Updating velocity for separation
    robot->xVel += close_dx * avoidanceFactor;
    robot->yVel += close_dy * avoidanceFactor;

    // Updating velocity for alignment

    if(numNeighbours > 0) {
        // Calculating the averages for velocity
        averageXVel = averageXVel / numNeighbours;
        averageYVel = averageYVel / numNeighbours;

        // Calculating the averages for position
        averageXPos = averageXPos / numNeighbours;
        averageYPos = averageYPos / numNeighbours;

        // Alignment
        robot->xVel += (robot->xVel - averageXVel) * alignmentFactor;
        robot->yVel += (robot->yVel - averageYVel) * alignmentFactor;

        // Cohesion
        robot->xVel += (averageXPos - robot->xPos) * cohesionFactor;
        robot->yVel += (averageYPos - robot->yPos) * cohesionFactor;
    }

    // Finding magnitude of linear velocity vector

    linearVel = sqrt(pow((robot->xVel), 2.0) + pow((robot->yVel), 2.0));

    // Computing the rotational velocity

    double newDirection = atan2(robot->yVel, robot->xVel);
    double angleDiff = newDirection - robot->GetPose().a;

    rotationalVel = angleDiff / (1/60);

    // Setting values for non-holonomic system
    robot->pos->SetSpeed(linearVel, 0, rotationalVel);

    return 0; // run again
}

// Calculating the distance between the current robot and the given pose of another robot
double SimpleRobot::CalculateDistance(Pose pose, SimpleRobot *robot) {
    Pose poseThis = robot->pos->GetPose();
    double xDiff = (double)(poseThis.x - pose.x);
    double yDiff = (double)(poseThis.y - pose.y);
    double distance = sqrt(abs((xDiff * xDiff) + (yDiff * yDiff)));
    return distance;
}

// Getter method to return the pose of the positional model
Pose SimpleRobot::GetPose() {
    return this->pos->GetPose();
}