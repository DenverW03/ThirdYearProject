#include "SimpleRobot.hh"
#include <stage.hh>
using namespace Stg;
using namespace SimpleRBT;
// Default constructor for init
SimpleRobot::SimpleRobot(){
    // No setup required in implicit constructor
};
// Constructor for the class with appropriate setup
SimpleRobot::SimpleRobot(ModelPosition *modelPos, int x, int y, SimpleRobot *robots) {
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
    this->pos->SetPose(Pose(x, y, 0, 0));
}
// Read the ranger data
int SimpleRobot::SensorUpdate(Model *, SimpleRobot* robot) {
    const std::vector<meters_t> &scan = robot->laser->GetSensors()[0].ranges;
    uint32_t sampleCount = scan.size();
    // std::cout << "Sample Count: " << sampleCount << "\n";
    if(sampleCount < 1) return 0; // not enough samples is not a legitimate reading for these purposes
    bool obstruction = false;
    double minFrontDistance = 2.0;
    double prescaler = 1 / scan[0]; // if it turns to a super close object should turn faster
    for(uint32_t i = 0; i < sampleCount; i++) {
        if(scan[i] < minFrontDistance) {
        obstruction = true;
        }
    }
    // printf("Obstruction: %d\n", obstruction);
    // std::cout << "Last Reading: " << scan[0] << "\n";
    // AVOID IF NECESSARY!
    if(obstruction) {
        robot->turnVel = 1 * prescaler;
        obstruction = false; // resetting
    }
    return 0;
}
// Position update function for stage
int SimpleRobot::PositionUpdate(Model *, SimpleRobot* robot) {
    printf("Distance from origin: %f\n", CalculateDistance(robot->robots[1].GetPose(), robot));
    /*Pose pose = robot->robots[1].GetPose();
    printf("Pose Robot [1]: [%.2f %.2f %.2f %.2f]\n", pose.x, pose.y, pose.z, pose.a);*/
    Pose pose = robot->pos->GetPose();
    printf("Pose This: [%.2f %.2f %.2f %.2f]\n", pose.x, pose.y, pose.z, pose.a);
    robot->pos->SetSpeed(robot->xVel, robot->yVel, robot->turnVel);
    robot->turnVel = 0;
    return 0; // run again
}
// Calculating the distance between the current robot and the given pose of another robot
double SimpleRobot::CalculateDistance(Pose pose, SimpleRobot *robot) {
    Pose poseThis = robot->pos->GetPose();
    double xDiff = (double)(poseThis.x - 0);
    double yDiff = (double)(poseThis.y - 0);
    double distance = sqrt(abs((xDiff * xDiff) + (yDiff * yDiff)));
    return distance;
}
// Getter method to return the pose of the positional model
Pose SimpleRobot::GetPose() {
    return this->pos->GetPose();
}