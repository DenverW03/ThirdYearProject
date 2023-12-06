#include <stage.hh>
#include "../common/src/SimpleRobot.hh"
using namespace Stg;
using namespace SimpleRBT;

SimpleRobot *robots = (SimpleRobot *) malloc(40 * sizeof(SimpleRobot));

int main(int argc, char* argv[]) {
    Init(&argc, &argv); // Initialising the stage library
    WorldGui world(635, 666, "Boids Simulation");
    world.Load(argv[1]);
    // After this comment the necessary robot spawns are added
    
    robots[0] = SimpleRobot((ModelPosition *)world.GetModel(argv[2]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[1] = SimpleRobot((ModelPosition *)world.GetModel(argv[3]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[2] = SimpleRobot((ModelPosition *)world.GetModel(argv[4]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[3] = SimpleRobot((ModelPosition *)world.GetModel(argv[5]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[4] = SimpleRobot((ModelPosition *)world.GetModel(argv[6]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[5] = SimpleRobot((ModelPosition *)world.GetModel(argv[7]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[6] = SimpleRobot((ModelPosition *)world.GetModel(argv[8]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[7] = SimpleRobot((ModelPosition *)world.GetModel(argv[9]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[8] = SimpleRobot((ModelPosition *)world.GetModel(argv[10]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[9] = SimpleRobot((ModelPosition *)world.GetModel(argv[11]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[10] = SimpleRobot((ModelPosition *)world.GetModel(argv[12]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[11] = SimpleRobot((ModelPosition *)world.GetModel(argv[13]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[12] = SimpleRobot((ModelPosition *)world.GetModel(argv[14]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[13] = SimpleRobot((ModelPosition *)world.GetModel(argv[15]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[14] = SimpleRobot((ModelPosition *)world.GetModel(argv[16]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[15] = SimpleRobot((ModelPosition *)world.GetModel(argv[17]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[16] = SimpleRobot((ModelPosition *)world.GetModel(argv[18]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[17] = SimpleRobot((ModelPosition *)world.GetModel(argv[19]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[18] = SimpleRobot((ModelPosition *)world.GetModel(argv[20]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[19] = SimpleRobot((ModelPosition *)world.GetModel(argv[21]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[20] = SimpleRobot((ModelPosition *)world.GetModel(argv[22]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[21] = SimpleRobot((ModelPosition *)world.GetModel(argv[23]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[22] = SimpleRobot((ModelPosition *)world.GetModel(argv[24]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[23] = SimpleRobot((ModelPosition *)world.GetModel(argv[25]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[24] = SimpleRobot((ModelPosition *)world.GetModel(argv[26]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[25] = SimpleRobot((ModelPosition *)world.GetModel(argv[27]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[26] = SimpleRobot((ModelPosition *)world.GetModel(argv[28]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[27] = SimpleRobot((ModelPosition *)world.GetModel(argv[29]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[28] = SimpleRobot((ModelPosition *)world.GetModel(argv[30]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[29] = SimpleRobot((ModelPosition *)world.GetModel(argv[31]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[30] = SimpleRobot((ModelPosition *)world.GetModel(argv[32]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[31] = SimpleRobot((ModelPosition *)world.GetModel(argv[33]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[32] = SimpleRobot((ModelPosition *)world.GetModel(argv[34]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[33] = SimpleRobot((ModelPosition *)world.GetModel(argv[35]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[34] = SimpleRobot((ModelPosition *)world.GetModel(argv[36]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[35] = SimpleRobot((ModelPosition *)world.GetModel(argv[37]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[36] = SimpleRobot((ModelPosition *)world.GetModel(argv[38]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[37] = SimpleRobot((ModelPosition *)world.GetModel(argv[39]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[38] = SimpleRobot((ModelPosition *)world.GetModel(argv[40]), Pose::Random(-4, 4, -4, 4), robots, 40);
    robots[39] = SimpleRobot((ModelPosition *)world.GetModel(argv[41]), Pose::Random(-4, 4, -4, 4), robots, 40);
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