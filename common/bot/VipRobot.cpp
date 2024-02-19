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
    this->pos->Subscribe();

    this->pos->SetPose(pose);

    this->pos->GoTo(Pose(8, 8, 0, 0));

    // Adding the circle visualizer
    this->cv = new CircleVIS::CircleVisualizer(0, 0, vipCircleRadius);
    this->pos->AddVisualizer(this->cv, true);
}

// Position update callback
int VipRobot::PositionUpdate(Model *, VipRobot* robot) {
    Pose pose = robot->pos->GetPose();

    if(round(pose.x) == -8 && round(pose.y) == 8) {
        robot->pos->GoTo(Pose(8, 8, 0, 0));
    }
    else if(round(pose.x) == 8 && round(pose.y) == 8) {
        robot->pos->GoTo(Pose(8, -8, 0, 0));
    }
    if(round(pose.x) == 8 && round(pose.y) == -8) {
        robot->pos->GoTo(Pose(-8, -8, 0, 0));
    }
    if(round(pose.x) == -8 && round(pose.y) == -8) {
        robot->pos->GoTo(Pose(-8, 8, 0, 0));
    }

    return 0;
}