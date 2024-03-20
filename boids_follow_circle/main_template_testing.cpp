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
            
            // std::ofstream dataFile("../testing/boids_follow_circle_results/output.csv", std::ios::app);
            // if (!dataFile.is_open()){
            //     std::cerr << "File failed to open: " << std::strerror(errno) << std::endl;
            //     break;
            // }
            // // If can quit early
            // for(int i=0; i<numRobots; i++) {
            //     // Printing results for robots that have not stalled
            //     if(robots[i].pos->Stalled()) {
            //         // Calculating the average distance
            //         double result2 = 0.0;
            //         for(double i : robots[i].testingDistances) {
            //             result2 += i;
            //         }
            //         result2 = result2 / robots[i].testingDistances.size();

            //         // Since these robots did not stall, 0 is the stall time
            //         dataFile << robots[i].id << "," << result2 << "," << 0 << std::endl;
            //         dataFile.close();
            //     }
            // }
        }
    }
    world.Quit();

    // Cleanup
    delete[] robots;
    // delete[] timeToStall;
    exit(0);
}