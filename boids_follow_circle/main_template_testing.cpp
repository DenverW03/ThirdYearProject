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
    // After this comment the necessary robot spawns are added
    
    world.Start();
    while(!world.TestQuit()) {
        if(!world.UpdateAll()) { // When UpdateAll() returns false, simulation continues
            world.Run();

            int counter = 0;
            for(int i=0; i<numRobots; i++) {
                if(robots[i].pos->Stalled()) {
                    counter += 1;
                }
            }
            if(counter == numRobots) world.Quit();
        };
    }
    
    for(int i=0; i<numRobots; i++) {
        std::ofstream dataFile("../testing/boids_follow_circle_results/stalled.csv", std::ios::app);
        if (!dataFile.is_open()){
            std::cerr << "File failed to open: " << std::strerror(errno) << std::endl;
            // File fails to open just end program
            break;
        }
        if(robots[i].pos->Stalled()) {
            dataFile << 1 << std::endl;
        }
        else {
            dataFile << 0 << std::endl;
        }
        dataFile.close();
    }

    // Cleanup
    free(robots);
    exit(0);
}