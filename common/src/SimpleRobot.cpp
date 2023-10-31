#include "SimpleRobot.hh"
#include <stage.hh>
using namespace Stg;
class SimpleRobot {
    public:
        ModelPosition *pos;
        ModelRanger *laser;
        double xVel;
        double yVel;
        double turnVel;
        
        // Function definitions
        int SensorUpdate(Model *, robot_t *robot);
        int PositionUpdate(Model *, robot_t *robot);

        SimpleRobot(ModelPosition *modelPos) {
            // Setting up positional model and callbacks
            this->pos = modelPos;
            this->pos->AddCallback(Model::CB_UPDATE, model_callback_t(PositionUpdate), this);
            this->laser = (ModelRanger *) (this->pos->GetChild("ranger:0"));
            this->laser->AddCallback(Model::CB_UPDATE, model_callback_t(SensorUpdate), robot);
            // Subscribing to callback updates
            this->pos->Subscribe();
            this->laser->Subscribe();
        }

        // read the ranger data
        int SensorUpdate(Model *) {
            const std::vector<meters_t> &scan = this->laser->GetSensors()[0].ranges;
            uint32_t sampleCount = scan.size();
            std::cout << "Sample Count: " << sampleCount << "\n";
            if(sampleCount < 1) return 0; // not enough samples is not a legitimate reading for these purposes
            bool obstruction = false;
            double minFrontDistance = 2.0;
            double prescaler = 1 / scan[0]; // if it turns to a super close object should turn faster
            for(uint32_t i = 0; i < sampleCount; i++) {
                if(scan[i] < minFrontDistance) {
                obstruction = true;
                }
            }
            printf("Obstruction: %d\n", obstruction);
            std::cout << "Last Reading: " << scan[0] << "\n";
            // AVOID IF NECESSARY!
            if(obstruction) {
                this->turnVel = 1 * prescaler;
                obstruction = false; // resetting
            }
            return 0;
        }
        // Position update function for stage
        int PositionUpdate(Model *) {
            Pose pose = this->pos->GetPose();
            printf("Pose: [%.2f %.2f %.2f %.2f]\n", pose.x, pose.y, pose.z, pose.a);
            this->pos->SetSpeed(this->xVel, this->yVel, this->turnVel);
            this->turnVel = 0;
            return 0; // run again
        }
    
}