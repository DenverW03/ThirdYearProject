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
    this->xVel = 1; // This velocity is the useful one for movement in this scenario
    this->yVel = 0; // y velocity is essentially useless to a robot bound for single direction movement (not omniwheel or ball)
    this->turnVel = 0; // This is the velocity used to control movement, positively anti-clockwise around the Z axis
    this->robots = robots;
    this->numRobots = numRobots;

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

        // Setting up some parameters
        bool obstruction = false;
        double minFrontDistance = 1.0;
        double prescaler = 1 / scan[0]; // if it turns to a super close object should turn faster
        
        // Going through the recorded samples to detect obstacles (current is a single recording per sensor update))
        for(uint32_t i = 0; i < sampleCount; i++) {
            if(scan[i] < minFrontDistance) {
                obstruction = true;
            }
            // Avoiding an obstacle
            if(obstruction) {
                Pose pose = robot->pos->GetPose();
                Pose laserPose = sensors[j].pose;
                double revAngle = laserPose.a + PI; // Invert direction by adding PI
                // Make sure the angle is within [0, 2*PI)
                if (revAngle >= 2 * PI) {
                    revAngle -= 2 * PI;
                }
                robot->pos->SetPose(Pose(pose.x, pose.y, pose.z, revAngle));
                obstruction = false; // resetting
            }
        }
    }
    return 0;
}
// Position update function for stage
int SimpleRobot::PositionUpdate(Model *, SimpleRobot* robot) {
    // Variables used as behaviour parameters
    double visionRange = 2; // The vision range for cohesion
    double avoidanceDistance = 1; // The vision range for avoidance
    double cohesion = 0.1; // Cohesion strength
    double avoidance = 0.2; // Avoidance strength

    // Variables used for calculating movement
    double numNeighbours = 0;
    double averageX = 0;
    double averageY = 0;
    double averageAngle = 0;
    double averageAngleTooClose = 0;
    double numTooClose = 0;
    
    // Looping through all the robots, numRobots given on instantiation of this positional model
    for(int i=0; i<robot->numRobots; i++) {
        if(robot->robots[i].pos == robot->pos) continue; // excluding self
        double distance = CalculateDistance(robot->robots[i].GetPose(), robot);
        if(distance <= visionRange) {
            numNeighbours += 1;
            averageX += robot->robots[i].GetPose().x;
            averageX += robot->robots[i].GetPose().y;
            averageAngle += robot->robots[i].GetPose().a;
        }
        if(distance <= avoidanceDistance) {
            numTooClose += 1;
            averageAngleTooClose += robot->robots[i].GetPose().a;
        }
    }
    if(numNeighbours > 0) {
        averageX = averageX / numNeighbours;
        averageY = averageY / numNeighbours;
        averageAngle = averageAngle / numNeighbours;
        averageAngleTooClose = averageAngleTooClose / numTooClose;
        // COHESION
        Pose position = robot->pos->GetPose();
        double vector[2];
        vector[0] = position.x - averageX; // getting a vector point transformation to the center of mass for the neighbours
        vector[1] = position.y - averageY;
        double goalAngle = atan2(vector[0], vector[1]);
        if (goalAngle - position.a > 0) { // If the goal angle is larger than the current angle turn positively
            robot->turnVel = -10 * cohesion * (goalAngle - position.a); // scale by cohesion factor and difference of angles
        }
        else {
            robot->turnVel = 10 * cohesion * (goalAngle - position.a);
        }
        // ALIGNMENT
        robot->pos->SetPose(Pose(position.x, position.y, position.z, averageAngle));
        // AVOIDANCE
    }
    else {
        robot->turnVel = 0; // if no neighbours then travel straight
    }
    robot->pos->SetSpeed(robot->xVel, robot->yVel, robot->turnVel);
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