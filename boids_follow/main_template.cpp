#include <stage.hh>
#include "./bot/ConvoyRobot.hh"
using namespace Stg;
using namespace ConvoyRBT;

ConvoyRobot *robots = (ConvoyRobot *) malloc(10 * sizeof(ConvoyRobot)); // Array length 10 to hold some robots during testing

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