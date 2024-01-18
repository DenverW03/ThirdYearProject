#include "SimpleRobot.hh"
#include <stage.hh>
#include <cmath>
#include <random>

using namespace Stg;
using namespace SimpleRBT;

// Default constructor for init
SimpleRobot::SimpleRobot(){
    // No setup required in implicit constructor
};

// Constructor for the class with appropriate setup
SimpleRobot::SimpleRobot(ModelPosition *modelPos, Pose pose, SimpleRobot *robots, int numRobots) {
    this->pos = nullptr;
    this->laser = nullptr;
    
    // Robot Velocity Values
    std::random_device rd;
    std::mt19937 generator = std::mt19937(rd());
    std::uniform_real_distribution<double> distribution(-3.0, 3.0);
    this->xVel = distribution(generator); // Generate random initial speed from -3 - 3
    this->yVel = distribution(generator); // Even though this bot is non holonomic, treat as holonomic until running bounding algorithm
    
    this->robots = robots; // Holds all the robots in an array
    this->numRobots = numRobots; // Holds the number of robots

    // Setting up positional model and callbacks
    this->pos = modelPos;
    this->pos->AddCallback(Model::CB_UPDATE, model_callback_t(PositionUpdate), this);
    this->laser = (ModelRanger *) (this->pos->GetChild("ranger:0"));
    this->laser->AddCallback(Model::CB_UPDATE, model_callback_t(SensorUpdate), this);
    
    // Subscribing to callback updates
    this->pos->Subscribe();
    this->laser->Subscribe();
    
    // Set the model initial position
    this->pos->SetPose(pose);
}

// Read the ranger data
int SimpleRobot::SensorUpdate(Model *, SimpleRobot* robot) {
    const std::vector<ModelRanger::Sensor> &sensors = robot->laser->GetSensors();

    // // Declaring some cumulative variables for the relative horizontal direction of the obstacle
    // double cumRight = 0;
    // double cumLeft = 0;

    // // Looping through all sensors
    // for(int j=0; j<sensors.size(); j++) {
    //     const std::vector<meters_t> &scan = robot->laser->GetSensors()[j].ranges;
    //     uint32_t sampleCount = scan.size();
    //     if(sampleCount < 1) continue;; // not enough samples is not a legitimate reading for these purposes

    //     // Check for obstacles in the front (multiply avoidance distance by 2 to extend time for avoiding obstacles)
    //     if (scan[0] < (avoidanceDistance * 2)) {

    //         // std::cout << "Reading Detected\r\n";
    //         // printf("Reading: %f\r\n", scan[0]);

    //         // Get the angles of the robot positional model and the sensor giving a reading
    //         double robotAngle = robot->pos->GetPose().a;
    //         double sensorAngle = robot->laser->GetSensors()[j].pose.a;

    //         // Decide on relative direction based on angle and add to cumulative count
    //         if(sensorAngle > robotAngle) cumLeft++;
    //         else if(sensorAngle <= robotAngle) cumRight++;
    //     }
    // }

    // if(!(cumRight && cumLeft == 0)){

    //     // printf("right: %f left: %f\r\n", cumRight, cumLeft);

    //     // Edit the rotational velocity of the robot based on the main side that the obstruction is on

    //     NHVelocities vels = CalculateNonHolonomic(robot->xVel, robot->yVel, robot);
        
    //     // In Stage4 angles are counter clockwise increasing
    //     if(cumRight > cumLeft) {
    //         robot->pos->SetTurnSpeed(-1 * fabs(vels.rotationalVel));
    //     }
    //     else {
    //         robot->pos->SetTurnSpeed(1 * fabs(vels.rotationalVel));
    //     }
    // }

    return 0;
}

// Position update function for stage
int SimpleRobot::PositionUpdate(Model *, SimpleRobot* robot) {

    // Separation
    double close_dx = 0;
    double close_dy = 0;

    // Alignment
    double averageXVel = 0;
    double averageYVel = 0;
    double numNeighbours = 0;

    // Cohesion
    double averageXPos = 0;
    double averageYPos = 0;

    // Non-holonmic velocity value struct
    NHVelocities vels;
    
    // Looping through all the robots, numRobots given on instantiation of this positional model
    for(int i = 0; i < robot->numRobots; i++) {
        // Exclude self
        if(robot->robots[i].pos == robot->pos) continue;

        // Get the distance of the robot positional model from this robot's positional model
        double distance = CalculateDistance(robot->robots[i].GetPose(), robot);

        // If the distance is within the avoidance range of the robot
        if(distance <= avoidanceDistance) {
            // Separation
            close_dx += robot->GetPose().x - robot->robots[i].GetPose().x;
            close_dy += robot->GetPose().y - robot->robots[i].GetPose().y;
        }

        // If the distance is within the vision range of the robot but outside avoidance range
        if(distance <= visionRange) {
            // Alignment
            averageXVel += robot->robots[i].xVel;
            averageYVel += robot->robots[i].yVel;
            numNeighbours += 1;

            // Cohesion
            averageXPos += robot->robots[i].GetPose().x;
            averageYPos += robot->robots[i].GetPose().y;
        }
    }

    // Updating velocity for separation

    robot->xVel += close_dx * avoidanceFactor;
    robot->yVel += close_dy * avoidanceFactor;

    // Updating velocity for alignment

    if(numNeighbours > 0) {
        // Calculating the averages for velocity
        averageXVel = averageXVel / numNeighbours;
        averageYVel = averageYVel / numNeighbours;

        // Calculating the averages for position
        averageXPos = averageXPos / numNeighbours;
        averageYPos = averageYPos / numNeighbours;

        // Alignment
        robot->xVel += (averageXVel - robot->xVel) * alignmentFactor;
        robot->yVel += (averageYVel - robot->yVel) * alignmentFactor;

        // Cohesion
        robot->xVel += (averageXPos - robot->GetPose().x) * cohesionFactor;
        robot->yVel += (averageYPos - robot->GetPose().y) * cohesionFactor;
    }

    vels = CalculateNonHolonomic(robot->xVel, robot->yVel, robot);

    // Setting values for non-holonomic system
    robot->pos->SetSpeed(vels.linearVel, 0, vels.rotationalVel);

    return 0; // run again
}


SimpleRobot::NHVelocities SimpleRobot::CalculateNonHolonomic(double xvel, double yvel, SimpleRobot *robot) {
    // Declaring a struct to hold velocity values
    NHVelocities vels;

    // Finding magnitude of linear velocity vector

    vels.linearVel = sqrt(pow((xvel), 2.0) + pow((yvel), 2.0));

    // Computing the rotational velocity

    double newDirection = atan2(yvel, xvel);
    double angleDiff = newDirection - robot->GetPose().a;

    vels.rotationalVel = angleDiff / (1/60);

    return vels;
}

// Calculating the distance between the current robot and the given pose of another robot
double SimpleRobot::CalculateDistance(Pose pose, SimpleRobot *robot) {
    Pose poseThis = robot->pos->GetPose();
    double xDiff = (double)(poseThis.x - pose.x);
    double yDiff = (double)(poseThis.y - pose.y);
    // Pythagoras theorem used to calculate the distance between two points
    double distance = sqrt(abs((xDiff * xDiff) + (yDiff * yDiff)));
    return distance;
}

// Getter method to return the pose of the positional model
Pose SimpleRobot::GetPose() {
    return this->pos->GetPose();
}