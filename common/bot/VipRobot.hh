#ifndef VIP_ROBOT
#define VIP_ROBOT
#include <stage.hh>  // Include the appropriate library for Stg
#include "../../boids_follow_circle/bot/CircleVisualizer.hh"

using namespace Stg;

namespace VipRBT {
    class VipRobot {
        #define vipCircleRadius 6
        
        public:
            ModelPosition *pos;
            
            // Due to the const nature of the inherited Visualizer class, to circumvent issues with copy constructors this is necessary.
            // Change the member variable type to a raw pointer to CircleVIS::CircleVisualizer
            CircleVIS::CircleVisualizer* cv;

            VipRobot();
            VipRobot(ModelPosition *modelPos, Pose pose);
            static int PositionUpdate(Model *, VipRobot *robot);
    };
}

#endif