#include <stage.hh>
#include "../common/src/SimpleRobot.hh"
using namespace Stg;
using namespace SimpleRBT;

SimpleRobot *robots = (SimpleRobot *) malloc(10 * sizeof(SimpleRobot)); // Array length 10 to hold some robots during testing

int main(int argc, char* argv[]) {
    Init(&argc, &argv); // Initialising the stage library
    WorldGui world(635, 666, "Boids Simulation");
    world.Load(argv[1]);
    // Add robots (Writing individual stuff is tedious so automating this with a .py script or something would be much better)
    robots[0] = SimpleRobot((ModelPosition *)world.GetModel(argv[2]), Pose::Random(-4, 4, -4, 4), robots);
    robots[1] = SimpleRobot((ModelPosition *)world.GetModel(argv[3]), Pose::Random(-4, 4, -4, 4), robots);
    robots[2] = SimpleRobot((ModelPosition *)world.GetModel(argv[4]), Pose::Random(-4, 4, -4, 4), robots);
    robots[3] = SimpleRobot((ModelPosition *)world.GetModel(argv[5]), Pose::Random(-4, 4, -4, 4), robots);
    robots[4] = SimpleRobot((ModelPosition *)world.GetModel(argv[6]), Pose::Random(-4, 4, -4, 4), robots);
    robots[5] = SimpleRobot((ModelPosition *)world.GetModel(argv[7]), Pose::Random(-4, 4, -4, 4), robots);
    robots[6] = SimpleRobot((ModelPosition *)world.GetModel(argv[8]), Pose::Random(-4, 4, -4, 4), robots);
    robots[7] = SimpleRobot((ModelPosition *)world.GetModel(argv[9]), Pose::Random(-4, 4, -4, 4), robots);
    robots[8] = SimpleRobot((ModelPosition *)world.GetModel(argv[10]), Pose::Random(-4, 4, -4, 4), robots);
    robots[9] = SimpleRobot((ModelPosition *)world.GetModel(argv[11]), Pose::Random(-4, 4, -4, 4), robots);
    world.Start();
    while(!world.TestQuit()) {
        if(!world.UpdateAll()) { // When UpdateAll() returns false, simulation continues
        //robot->pos->GoTo( 3.0 * flippyFlip, -7.0 * flippyFlip, 2.0 );
        world.Run();
        };
    }
    // Cleanup
    free(robots);
    exit(0);
}