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
SimpleRobot::SimpleRobot(ModelPosition *modelPos, Pose pose, SimpleRobot *robots) {
    this->pos = nullptr;
    this->laser = nullptr;
    this->xVel = 1;
    this->yVel = 1;
    this->turnVel = 0;
    this->robots = robots;
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
    const std::vector<meters_t> &scan = robot->laser->GetSensors()[0].ranges;
    uint32_t sampleCount = scan.size();
    if(sampleCount < 1) return 0; // not enough samples is not a legitimate reading for these purposes
    bool obstruction = false;
    double minFrontDistance = 2.0;
    double prescaler = 1 / scan[0]; // if it turns to a super close object should turn faster
    for(uint32_t i = 0; i < sampleCount; i++) {
        if(scan[i] < minFrontDistance) {
        obstruction = true;
        }
    }
    // Avoiding an obstacle
    if(obstruction) {
        //robot->turnVel = 2 * prescaler; // was originally 1 * prescaler
        obstruction = false; // resetting
    }
    return 0;
}
// Position update function for stage
int SimpleRobot::PositionUpdate(Model *, SimpleRobot* robot) {
    double visionRange = 3; // The vision range for the boid, should be moved into a class variable
    double avoidanceDistance = 0.5;
    double cohesion = 0.1;
    double avoidance = 0.2;
    // Variables used for calculating movement
    double numNeighbours = 0;
    double averageX = 0;
    double averageY = 0;
    double averageAngle = 0;
    for(int i=0; i<3; i++) {
        if(robot->robots[i].pos == robot->pos) continue; // excluding self
        double distance = CalculateDistance(robot->robots[i].GetPose(), robot);
        if(distance < visionRange) {
            numNeighbours += 1;
            averageX += robot->robots[i].GetPose().x;
            averageX += robot->robots[i].GetPose().y;
            averageAngle += robot->robots[i].GetPose().a;
        }
    }
    if(numNeighbours > 0) {
        averageX = averageX / numNeighbours;
        averageY = averageY / numNeighbours;
        averageAngle = averageAngle / numNeighbours;
        /*double thisAngle = robot->pos->GetPose().a;
        // Change the turn velocity based on the difference between the current facing angle and average angle of the flock
        double difference = thisAngle - averageAngle;
        double direction = -1;
        if(difference > PI) direction = 1;
        robot->turnVel = direction * difference * cohesion;*/
        robot->pos->GoTo(averageX, averageY, averageAngle);
    }
    else{
        robot->pos->SetSpeed(robot->xVel, robot->yVel, robot->turnVel);
    }
    //robot->pos->SetSpeed(robot->xVel, robot->yVel, robot->turnVel);
    //robot->turnVel = 0; // Resetting for obstacle avoidance
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