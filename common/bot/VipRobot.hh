#ifndef VIP_ROBOT
#define VIP_ROBOT
#include <stage.hh>  // Include the appropriate library for Stg

using namespace Stg;

namespace VipRBT {
    class VipRobot {  
    public:
        VipRobot();
        VipRobot(ModelPosition *modelPos, Pose pose);
        static int PositionUpdate(Model *, VipRobot *robot);
    };
}

#endif
