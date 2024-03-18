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
SimpleRobot::SimpleRobot(ModelPosition *modelPos, Pose pose) {
    this->pos = nullptr;

    // Setting the space for the camera array using C++ convention rather than malloc
    this->cameras = new ModelBlobfinder * [camCount];
    
    // Robot Velocity Values
    std::random_device rd;
    std::mt19937 generator = std::mt19937(rd());
    std::uniform_real_distribution<double> distribution(-3.0, 3.0);
    this->xVel = distribution(generator); // Generate random initial speed from -3 - 3
    this->yVel = distribution(generator); // Even though this bot is non holonomic, treat as holonomic until running bounding algorithm

    // Initialising the boid data values to zero
    this->boidData.closeDx = 0;
    this->boidData.closeDy = 0;
    this->boidData.closeDxObs = 0;
    this->boidData.closeDyObs = 0;
    this->boidData.averageXPos = 0;
    this->boidData.averageYPos = 0;
    this->boidData.numNeighbours = 0;

    // Setting up positional model and callbacks
    this->pos = modelPos;
    this->pos->AddCallback(Model::CB_UPDATE, model_callback_t(PositionUpdate), this);

    for(int i = 0; i<camCount; i++) {
        this->cameras[i] = (ModelBlobfinder *) (this->pos->GetChild("blobfinder:" + std::to_string(i)));

        // Using a struct to pass extra data to stage AddCallback() function
        SensorInputData* data = (SensorInputData *) malloc(sizeof(SensorInputData));;
        data->robot = this;
        data->bf = cameras[i];
        data->num = i;
        void* dataPtr = static_cast<void*>(data);

        this->cameras[i]->AddCallback(Model::CB_UPDATE, model_callback_t(SensorUpdate), dataPtr); // SHOULD ADD EXTRA PARAM FOR INDEX camera[i] for example
        this->cameras[i]->Subscribe();
    }
    
    // Subscribing to callback updates
    this->pos->Subscribe();
    
    // Set the model initial position
    this->pos->SetPose(pose);
    NHVelocities nonHolonomic = CalculateNonHolonomic(this->xVel, this->yVel, this);
    this->pos->SetSpeed(nonHolonomic.linearVel, 0, nonHolonomic.rotationalVel);
}

// Sensor update callback
int SimpleRobot::SensorUpdate(Model *, SensorInputData* data) {
    SimpleRobot* robot = data->robot;
    ModelBlobfinder* blobfinder = data->bf;

    // Getting the blobs found in the blobfinder model
    const std::vector<ModelBlobfinder::Blob> &blobs = blobfinder->GetBlobs();

    // Running the boids value generation algorithm for every blob
    for(const ModelBlobfinder::Blob blob : blobs){
        int r = blob.color.r * 255;
        int g = blob.color.g * 255;
        int b = blob.color.b * 255;
        int a = blob.color.a * 255;

        int full = (r << 24) | (g << 16) | (b << 8) | a;

        double distance = blob.range;

        // Case for handling obstacles and case for handling other bots
        switch(full) {
            case black: {
                // // Need distance to fall equal to or lower than the obstacle avoidance distance
                if (distance > avoidObstructionDistance) break;

                // Add this blob to the close stacks
                Pose pose = robot->GetPose();
                auto position = CalculatePosition(robot->angles[data->num], pose, distance);
                robot->boidData.closeDxObs -= pose.x - position.first;
                robot->boidData.closeDyObs -= pose.y - position.second;
                break;
            }
            case blue: {
                // Guard clause to avoid wasting compute
                if(distance > visionRange) break;
                
                // Get the blob real world position
                Pose pose = robot->GetPose();
                auto position = CalculatePosition(robot->angles[data->num], pose, distance);

                // If the distance is within the avoidance range of the robot

                // Separation (prior guard clause already confirmed bot to be within avoidance distance)
                if(distance <= avoidanceDistance) {
                    robot->boidData.closeDx += robot->GetPose().x - position.first;
                    robot->boidData.closeDy += robot->GetPose().y - position.second;
                }

                // If the distance is within the vision range of the robot but outside avoidance range
                if(distance <= visionRange) {
                    // // Alignment
                    // averageXVel += robot->robots[i].xVel;
                    // averageYVel += robot->robots[i].yVel;

                    robot->boidData.numNeighbours += 1;

                    // Cohesion
                    robot->boidData.averageXPos += position.first;
                    robot->boidData.averageYPos += position.second;
                }

                break;
            }
        }
    }

    return 0;
}

std::pair<double, double> SimpleRobot::CalculatePosition(double a, Pose pose, double distance) {
    // Get angle of sensor on bot (if negative just add (pi - absolute value) to pi to get positive angle representation)
    double theta = a * (M_PI / 180);
    if (theta < 0) theta = (2 * M_PI) - abs(theta);

    // Get obstacle distance (hypotenuse)
    double hyp = distance;

    // Using a composite angle for real world sensor angle
    // Pose pose = robot->GetPose();
    double botAngle = pose.a;
    if (botAngle < 0) botAngle = (2 * M_PI) - abs(botAngle);
    double compositeAngle = botAngle + theta;
    if (compositeAngle > (2 * M_PI)) compositeAngle -= (2 * M_PI);

    // Use trigonometry to deduce the position of the obstacle in vector from bot, using abs value to decide direction in post
    double opp = hyp * sin(compositeAngle); // y
    double adj = hyp * cos(compositeAngle); // x

    // Obstacle position calculations
    double xpos = pose.x + adj;
    double ypos = pose.y + opp;

    return std::make_pair(xpos, ypos);
}

// Position update function for stage (necessary for the bot to actually move)
int SimpleRobot::PositionUpdate(Model *, SimpleRobot* robot) {
    // Avoidance

    // For other bots
    robot->xVel += robot->boidData.closeDx * avoidanceFactor;
    robot->yVel += robot->boidData.closeDy * avoidanceFactor;

    // For obstacles
    robot->xVel -= robot->boidData.closeDxObs * avoidObstructionFactor;
    robot->yVel -= robot->boidData.closeDyObs * avoidObstructionFactor;

    // Cohesion and Alignment

    if(robot->boidData.numNeighbours > 0) {
        // // Calculating the averages for velocity
        // averageXVel = averageXVel / numNeighbours;
        // averageYVel = averageYVel / numNeighbours;

        // Calculating the averages for position
        robot->boidData.averageXPos = robot->boidData.averageXPos / robot->boidData.numNeighbours;
        robot->boidData.averageYPos = robot->boidData.averageYPos / robot->boidData.numNeighbours;

        // // Alignment
        // robot->xVel += (averageXVel - robot->xVel) * alignmentFactor;
        // robot->yVel += (averageYVel - robot->yVel) * alignmentFactor;

        // Cohesion
        robot->xVel += (robot->boidData.averageXPos - robot->GetPose().x) * cohesionFactor;
        robot->yVel += (robot->boidData.averageYPos - robot->GetPose().y) * cohesionFactor;
    }

    // Diagnostic printing

    HVelocities vels2 = CalculateHolonomic(robot->pos->GetVelocity().x, robot->pos->GetVelocity().a, robot);


    NHVelocities nonHolonomic = CalculateNonHolonomic(robot->xVel, robot->yVel, robot);
    robot->pos->SetSpeed(nonHolonomic.linearVel, 0, nonHolonomic.rotationalVel);

    // Resetting the boid data values
    robot->boidData.closeDx = 0;
    robot->boidData.closeDy = 0;
    robot->boidData.closeDxObs = 0;
    robot->boidData.closeDyObs = 0;
    robot->boidData.averageXPos = 0;
    robot->boidData.averageYPos = 0;
    robot->boidData.numNeighbours = 0;


    // Setting stored velocity to the real velocity so it doesn't grow too large in magnitude
    robot->xVel = vels2.xvel;
    robot->yVel = vels2.yvel;


    return 0;
}

SimpleRobot::HVelocities SimpleRobot::CalculateHolonomic(double linearvel, double turnvel, SimpleRobot *robot) {
    // Declaring a struct to hold velocity values
    HVelocities vels;

    // Calculating the angle difference

    // double angleDiff = turnvel * (1.0/60.0);

    // Calculating the new direction

    // double newDirection = angleDiff + robot->GetPose().a;
    double newDirection = robot->GetPose().a;

    // Using trig to convert from polar velocity to cartesian

    vels.xvel = linearvel * cos(newDirection);
    vels.yvel = linearvel * sin(newDirection);

    return vels;
}

SimpleRobot::NHVelocities SimpleRobot::CalculateNonHolonomic(double xvel, double yvel, SimpleRobot *robot) {
    // Declaring a struct to hold velocity values
    NHVelocities vels;

    // Finding magnitude of linear velocity vector

    vels.linearVel = sqrt(pow((xvel), 2.0) + pow((yvel), 2.0));

    // Computing the rotational velocity

    double newDirection = atan2(yvel, xvel);
    double angleDiff = newDirection - robot->GetPose().a;

    vels.rotationalVel = angleDiff / (1.0/60.0);

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