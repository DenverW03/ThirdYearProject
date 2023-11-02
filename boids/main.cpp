#include <stage.hh>
#include "../common/src/SimpleRobot.hh"
using namespace Stg;
using namespace SimpleRBT;

SimpleRobot robots[10]; // Array length 10 to hold some robots during testing

int main(int argc, char* argv[]) {
    Init(&argc, &argv); // Initialising the stage library
    WorldGui world(635, 666, "Boids Simulation");
    world.Load(argv[1]);
    // Add robots
    robots[0] = SimpleRobot((ModelPosition *)world.GetModel(argv[2]));
    world.Start();
    while(!world.TestQuit()) {
        if(!world.UpdateAll()) { // When UpdateAll() returns false, simulation continues
        //robot->pos->GoTo( 3.0 * flippyFlip, -7.0 * flippyFlip, 2.0 );
        world.Run();
        };
    }
    // Cleanup
    exit(0);
}