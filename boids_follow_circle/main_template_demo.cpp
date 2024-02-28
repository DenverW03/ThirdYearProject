#include <stage.hh>
#include "./bot/ConvoyRobot.hh"
#include "../common/bot/VipRobot.hh"
#include <iostream>
#include <fstream>
#include <sstream>
using namespace Stg;
using namespace ConvoyRBT;


int main(int argc, char* argv[]) {
    Init(&argc, &argv); // Initialising the stage library
    WorldGui world(635, 666, "Simulation");
    world.Load(argv[1]);
    // After this comment the necessary robot spawns are added
    
    world.Start();
    while(!world.TestQuit()) {
        if(!world.UpdateAll()) { // When UpdateAll() returns false, simulation continues
            world.Run();
        };
    }

    // Cleanup
    free(robots);
    exit(0);
}