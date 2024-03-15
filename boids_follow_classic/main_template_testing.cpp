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
    // for(int i=0; i<numRobots; i++) {
    //     robots[i] = ConvoyRobot((ModelPosition *)world.GetModel(argv[i + 3]), Pose::Random(-12, 12, -12, 12));
    //     printf("Added: %d", i);
    // }
    
    
    world.Start();
    while(!world.TestQuit()) {
        if(!world.UpdateAll()) { // When UpdateAll() returns false, simulation continues
            world.Run();
            
            // If can quit early
            int counter = 0;
            for(int i=0; i<numRobots; i++) {
                if(robots[i].pos->Stalled()) {
                    // Storing the time to stall in seconds
                    // timeToStall[i] = world.SimTimeNow();
                    // printf("Time to stall %llu\r\n", world.SimTimeNow());
                    counter += 1;
                }
            }
            if(counter == numRobots) world.Quit();
        };
    }
    world.Quit();

    // Cleanup
    delete[] robots;
    // delete[] timeToStall;
    exit(0);
}