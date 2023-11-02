#include "SimpleRobot.hh"
#include <stage.hh>
using namespace Stg;
using namespace SimpleRBT;
// Default constructor for init
SimpleRobot::SimpleRobot(){
    pos = nullptr;
    laser = nullptr;
    xVel = 1;
    yVel = 1;
    turnVel = 0;
};
// Constructor for the class with appropriate setup
SimpleRobot::SimpleRobot(ModelPosition *modelPos) {
    // Setting up positional model and callbacks
    this->pos = modelPos;
    this->pos->AddCallback(Model::CB_UPDATE, model_callback_t(PositionUpdate), this);
    this->laser = (ModelRanger *) (this->pos->GetChild("ranger:0"));
    this->laser->AddCallback(Model::CB_UPDATE, model_callback_t(SensorUpdate), this);
    // Subscribing to callback updates
    this->pos->Subscribe();
    this->laser->Subscribe();
}
// Read the ranger data
int SimpleRobot::SensorUpdate(Model *, SimpleRobot* robot) {
    const std::vector<meters_t> &scan = robot->laser->GetSensors()[0].ranges;
    uint32_t sampleCount = scan.size();
    std::cout << "Sample Count: " << sampleCount << "\n";
    if(sampleCount < 1) return 0; // not enough samples is not a legitimate reading for these purposes
    bool obstruction = false;
    double minFrontDistance = 2.0;
    double prescaler = 1 / scan[0]; // if it turns to a super close object should turn faster
    for(uint32_t i = 0; i < sampleCount; i++) {
        if(scan[i] < minFrontDistance) {
        obstruction = true;
        }
    }
    printf("Obstruction: %d\n", obstruction);
    std::cout << "Last Reading: " << scan[0] << "\n";
    // AVOID IF NECESSARY!
    if(obstruction) {
        robot->turnVel = 1 * prescaler;
        obstruction = false; // resetting
    }
    return 0;
}
// Position update function for stage
int SimpleRobot::PositionUpdate(Model *, SimpleRobot* robot) {
    Pose pose = robot->pos->GetPose();
    printf("Pose: [%.2f %.2f %.2f %.2f]\n", pose.x, pose.y, pose.z, pose.a);
    robot->pos->SetSpeed(robot->xVel, robot->yVel, robot->turnVel);
    robot->turnVel = 0;
    return 0; // run again
}