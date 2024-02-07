#include "VipRobot.hh"
#include <stage.hh>
#include <cmath>
#include <random>

using namespace Stg;
using namespace VipRBT;

// Default constructor for init
VipRobot::VipRobot(){
    // No setup required in implicit constructor
};

// Constructor for the class with appropriate setup
VipRobot::VipRobot(ModelPosition *modelPos, Pose pose) {

    // Adding the bot to the world
    this->pos = modelPos;
    this->pos->AddCallback(Model::CB_UPDATE, model_callback_t(PositionUpdate), this);

    this->pos->SetPose(pose);

}

// // Sensor update callback
// int VipRobot::SensorUpdate(Model *, VipRobot* robot) {

//     return 0;
// }

// Position update function for stage (necessary for the bot to actually move)
int VipRobot::PositionUpdate(Model *, VipRobot* robot) {

    robot->pos->GoTo(Pose(8, 8, 0, 0));

    return 0;
}