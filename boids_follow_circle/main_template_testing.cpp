#include <stage.hh>
#include "./bot/ConvoyRobot.hh"
#include "../common/bot/VipRobot.hh"
#include <iostream>
#include <fstream>
#include <sstream>

using namespace Stg;
using namespace ConvoyRBT;

#define numRobots insert

int main(int argc, char* argv[]) {
    Init(&argc, &argv); // Initialising the stage library
    World world("Simulation");
    world.Load(argv[1]);

    // Adding the robots for the simulation
    ConvoyRobot *robots = new ConvoyRobot[numRobots];
    VipRBT::VipRobot vip = VipRBT::VipRobot((ModelPosition *)world.GetModel(argv[2]), Pose(-8, 8, 0, 0));
    
    world.Start();
    while(!world.TestQuit()) {
        if(!world.UpdateAll()) { // When UpdateAll() returns false, simulation continues
            world.Run();
        }
    }
    world.Quit();

    // Cleanup
    delete[] robots;
    // delete[] timeToStall;
    exit(0);
}