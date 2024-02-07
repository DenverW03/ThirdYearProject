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

}

// // Sensor update callback
// int VipRobot::SensorUpdate(Model *, VipRobot* robot) {

//     return 0;
// }

// Position update function for stage (necessary for the bot to actually move)
int VipRobot::PositionUpdate(Model *, VipRobot* robot) {

    return 0;
}