#include <stage.hh>
#include "./bot/ConvoyRobot.hh"
#include "../common/bot/VipRobot.hh"
using namespace Stg;
using namespace ConvoyRBT;
ConvoyRobot *robots = (ConvoyRobot *) malloc(30 * sizeof(ConvoyRobot));

int main(int argc, char* argv[]) {
    Init(&argc, &argv); // Initialising the stage library
    WorldGui world(635, 666, "Simulation");
    world.Load(argv[1]);
    // After this comment the necessary robot spawns are added
    VipRBT::VipRobot vip = VipRBT::VipRobot((ModelPosition *)world.GetModel(argv[2]), Pose(-8, 8, 0, 0));
    robots[0] = ConvoyRobot((ModelPosition *)world.GetModel(argv[3]), Pose::Random(-12, 12, -12, 12));
    robots[1] = ConvoyRobot((ModelPosition *)world.GetModel(argv[4]), Pose::Random(-12, 12, -12, 12));
    robots[2] = ConvoyRobot((ModelPosition *)world.GetModel(argv[5]), Pose::Random(-12, 12, -12, 12));
    robots[3] = ConvoyRobot((ModelPosition *)world.GetModel(argv[6]), Pose::Random(-12, 12, -12, 12));
    robots[4] = ConvoyRobot((ModelPosition *)world.GetModel(argv[7]), Pose::Random(-12, 12, -12, 12));
    robots[5] = ConvoyRobot((ModelPosition *)world.GetModel(argv[8]), Pose::Random(-12, 12, -12, 12));
    robots[6] = ConvoyRobot((ModelPosition *)world.GetModel(argv[9]), Pose::Random(-12, 12, -12, 12));
    robots[7] = ConvoyRobot((ModelPosition *)world.GetModel(argv[10]), Pose::Random(-12, 12, -12, 12));
    robots[8] = ConvoyRobot((ModelPosition *)world.GetModel(argv[11]), Pose::Random(-12, 12, -12, 12));
    robots[9] = ConvoyRobot((ModelPosition *)world.GetModel(argv[12]), Pose::Random(-12, 12, -12, 12));
    robots[10] = ConvoyRobot((ModelPosition *)world.GetModel(argv[13]), Pose::Random(-12, 12, -12, 12));
    robots[11] = ConvoyRobot((ModelPosition *)world.GetModel(argv[14]), Pose::Random(-12, 12, -12, 12));
    robots[12] = ConvoyRobot((ModelPosition *)world.GetModel(argv[15]), Pose::Random(-12, 12, -12, 12));
    robots[13] = ConvoyRobot((ModelPosition *)world.GetModel(argv[16]), Pose::Random(-12, 12, -12, 12));
    robots[14] = ConvoyRobot((ModelPosition *)world.GetModel(argv[17]), Pose::Random(-12, 12, -12, 12));
    robots[15] = ConvoyRobot((ModelPosition *)world.GetModel(argv[18]), Pose::Random(-12, 12, -12, 12));
    robots[16] = ConvoyRobot((ModelPosition *)world.GetModel(argv[19]), Pose::Random(-12, 12, -12, 12));
    robots[17] = ConvoyRobot((ModelPosition *)world.GetModel(argv[20]), Pose::Random(-12, 12, -12, 12));
    robots[18] = ConvoyRobot((ModelPosition *)world.GetModel(argv[21]), Pose::Random(-12, 12, -12, 12));
    robots[19] = ConvoyRobot((ModelPosition *)world.GetModel(argv[22]), Pose::Random(-12, 12, -12, 12));
    robots[20] = ConvoyRobot((ModelPosition *)world.GetModel(argv[23]), Pose::Random(-12, 12, -12, 12));
    robots[21] = ConvoyRobot((ModelPosition *)world.GetModel(argv[24]), Pose::Random(-12, 12, -12, 12));
    robots[22] = ConvoyRobot((ModelPosition *)world.GetModel(argv[25]), Pose::Random(-12, 12, -12, 12));
    robots[23] = ConvoyRobot((ModelPosition *)world.GetModel(argv[26]), Pose::Random(-12, 12, -12, 12));
    robots[24] = ConvoyRobot((ModelPosition *)world.GetModel(argv[27]), Pose::Random(-12, 12, -12, 12));
    robots[25] = ConvoyRobot((ModelPosition *)world.GetModel(argv[28]), Pose::Random(-12, 12, -12, 12));
    robots[26] = ConvoyRobot((ModelPosition *)world.GetModel(argv[29]), Pose::Random(-12, 12, -12, 12));
    robots[27] = ConvoyRobot((ModelPosition *)world.GetModel(argv[30]), Pose::Random(-12, 12, -12, 12));
    robots[28] = ConvoyRobot((ModelPosition *)world.GetModel(argv[31]), Pose::Random(-12, 12, -12, 12));
    robots[29] = ConvoyRobot((ModelPosition *)world.GetModel(argv[32]), Pose::Random(-12, 12, -12, 12));
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