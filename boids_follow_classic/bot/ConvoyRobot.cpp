#include "ConvoyRobot.hh"
#include <stage.hh>
#include <cmath>
#include <random>
#include <thread>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace Stg;
using namespace ConvoyRBT;

// Default constructor for init
ConvoyRobot::ConvoyRobot(){
    // No setup required in implicit constructor
};

// Constructor for the class with appropriate setup
ConvoyRobot::ConvoyRobot(ModelPosition *modelPos, Pose pose, int id) {
    this->id = id;
    this->pos = nullptr;

    this->stack = (struct VipVelocityNode*)malloc(sizeof(struct VipVelocityNode));

    // Setting the space for the camera array using C++ convention rather than malloc
    this->cameras = new ModelBlobfinder * [camCount];
    
    // Robot Velocity Values
    std::random_device rd;
    std::mt19937 generator = std::mt19937(rd());
    std::uniform_real_distribution<double> distribution(-3.0, 3.0);
    this->xVel = distribution(generator); // Generate random initial speed from -3 - 3
    this->yVel = distribution(generator); // Even though this bot is non holonomic, treat as holonomic until running bounding algorithm

    // Initialising the boid data values to zero
    this->boidData.separateX = 0;
    this->boidData.separateY = 0;
    this->boidData.separateXObs = 0;
    this->boidData.separateYObs = 0;
    this->boidData.averageXPos = 0;
    this->boidData.averageYPos = 0;
    this->boidData.numNeighbours = 0;

    // Making the VIP last position stack empty to begin
    push(0, 0, this);
    push(0, 0, this);

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

        this->cameras[i]->AddCallback(Model::CB_UPDATE, model_callback_t(SensorUpdate), dataPtr);
        this->cameras[i]->Subscribe();
    }
    
    // Subscribing to callback updates
    this->pos->Subscribe();
    
    // Set the model initial position
    this->pos->SetPose(pose);
    NHVelocities nonHolonomic = CalculateNonHolonomic(this->xVel, this->yVel, this);
    this->pos->SetSpeed(nonHolonomic.linearVel, 0, nonHolonomic.rotationalVel);

    printf("Running\r\n");

    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
    long long milliseconds_count = milliseconds.count();
    this->startTime = static_cast<unsigned long>(milliseconds_count);
    this->lastSysTime = this->startTime;
}

// Sensor update callback
int ConvoyRobot::SensorUpdate(Model *, SensorInputData* data) {
    ConvoyRobot* robot = data->robot;
    ModelBlobfinder* blobfinder = data->bf;

    // Getting the blobs found in the blobfinder model
    const std::vector<ModelBlobfinder::Blob> &blobs = blobfinder->GetBlobs();

    // Running the boids value generation algorithm for every blob
    for(const ModelBlobfinder::Blob blob : blobs){
        int r = blob.color.r * 255;
        int g = blob.color.g * 255;
        int b = blob.color.b * 255;
        int a = blob.color.a * 255;

        unsigned int full = (r << 24) | (g << 16) | (b << 8) | a; // unsigned to hold full 32 bit colour

        double distance = blob.range;

        // Using these varibales so that when sight of VIP lost velocity is not still affected
        double vipEffectX = 0;
        double vipEffectY = 0;

        // Case for handling obstacles and case for handling other bots
        switch(full) {
            case black: { // OBSTACLES
                // Need distance to fall equal to or lower than the obstacle avoidance distance
                if (distance > avoidObstructionDistance) break;

                // Add this blob to the close stacks
                Pose pose = robot->GetPose();
                auto position = CalculatePosition(robot->angles[data->num], pose, distance);
                robot->boidData.separateXObs -= pose.x - position.first;
                robot->boidData.separateYObs -= pose.y - position.second;
                break;
            }
            case blue: { // OTHER CONVOY BOTS
                // Guard clause to avoid wasting compute
                if(distance > visionRange) break;
                
                // Get the blob real world position
                Pose pose = robot->GetPose();
                auto position = CalculatePosition(robot->angles[data->num], pose, distance);

                // If the distance is within the avoidance range of the robot

                // Separation (prior guard clause already confirmed bot to be within avoidance distance)
                if(distance <= avoidanceDistance) {
                    robot->boidData.separateX += pose.x - position.first;
                    robot->boidData.separateY += pose.y - position.second;
                }

                // If the distance is within the vision range of the robot but outside avoidance range
                if(distance <= visionRange) {
                    robot->boidData.numNeighbours += 1;

                    // Cohesion
                    robot->boidData.averageXPos += position.first;
                    robot->boidData.averageYPos += position.second;
                }
                break;
            }
            case red: { // THE VIPs

                // Getting the stage simulation time difference which has to be adjusted for the time scale
                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
                long long ms_count = ms.count();
                unsigned long timeDiff = (static_cast<unsigned long>(ms_count) - robot->lastSysTime) * timeScale;

                // For red VIP going to have the same vision range as for fellow convoy bots
                if(distance > visionRange) break;

                Pose pose = robot->GetPose();
                auto position = CalculatePosition(robot->angles[data->num], pose, distance);

                // Alignment, updating the vip positions
                vipEffectX = position.first;
                vipEffectY = position.second;

                // Min distance to VIP
                if(distance <= vipMinDistance) {
                    robot->boidData.separateX += pose.x - position.first;
                    robot->boidData.separateY += pose.y - position.second;
                }
                
                // Max distance from VIP
                if(distance <= vipMaxDistance) {
                    robot->boidData.numNeighbours += (1 * vipCohesionMultiplier);
                
                    robot->boidData.averageXPos += position.first * vipCohesionMultiplier;
                    robot->boidData.averageYPos += position.second * vipCohesionMultiplier;
                }
                
                // Trying to keep the convoy in bounds
                if(distance > vipMaxDistance) {
                    robot->boidData.separateX -= (pose.x - position.first);
                    robot->boidData.separateY -= (pose.y - position.second);
                
                }

                // If it has been over polling rate, take new position estimate
                if(timeDiff > velocityPollingRate) {
                    push(vipEffectX, vipEffectY, robot);
                    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
                    long long milliseconds_count = milliseconds.count();
                    robot->lastSysTime = static_cast<unsigned long>(milliseconds_count);
                }
                break;
            }
        }
    }
    return 0;
}

// Position update function for stage (necessary for the bot to actually move)
int ConvoyRobot::PositionUpdate(Model *, ConvoyRobot* robot) {
    // Check for stalling (not working)

    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
    long long ms_count = ms.count();
    // Testing for when robot has crashed, or if it hasn't printing the distances before the end of the simulation
    if((robot->pos->Stalled() || (static_cast<unsigned long>(ms_count) - robot->startTime) * (timeScale / 2) >= 580000) && testing) {
        // Unsubscribe from callback so no longer called
        robot->pos->Unsubscribe();

        // Call data output function for bot stalling
        TestingStall(robot);

        // Exit so no wasting compute
        return 0;
    }

    // For other bots
    robot->xVel += robot->boidData.separateX * avoidanceFactor;
    robot->yVel += robot->boidData.separateY * avoidanceFactor;

    // For obstacles
    robot->xVel -= robot->boidData.separateXObs * avoidObstructionFactor;
    robot->yVel -= robot->boidData.separateYObs * avoidObstructionFactor;

    // For the VIP
    robot->xVel -= robot->boidData.separateX * vipSeparationMultiplier;
    robot->yVel -= robot->boidData.separateY * vipSeparationMultiplier;

    // Cohesion and Alignment

    if(robot->boidData.numNeighbours > 0) {
        // Calculating the averages for position
        robot->boidData.averageXPos = robot->boidData.averageXPos / robot->boidData.numNeighbours;
        robot->boidData.averageYPos = robot->boidData.averageYPos / robot->boidData.numNeighbours;

        // Cohesion
        robot->xVel += (robot->boidData.averageXPos - robot->GetPose().x) * cohesionFactor;
        robot->yVel += (robot->boidData.averageYPos - robot->GetPose().y) * cohesionFactor;

        // Alignment of convoy bot with VIP bot
        if(robot->stack->second != nullptr){
            double dx = (robot->stack->xpos - robot->stack->second->xpos);
            double dy = (robot->stack->ypos - robot->stack->second->ypos);

            robot->xVel += (robot->xVel - dx) * vipAlignmentMultiplier;
            robot->yVel += (robot->yVel - dx) * vipAlignmentMultiplier;
        }
    }

    // Diagnostic printing

    HVelocities vels2 = CalculateHolonomic(robot->pos->GetVelocity().x, robot->pos->GetVelocity().a, robot);


    NHVelocities nonHolonomic = CalculateNonHolonomic(robot->xVel, robot->yVel, robot);
    robot->pos->SetSpeed(nonHolonomic.linearVel, 0, nonHolonomic.rotationalVel);

    // Resetting the boid data values
    robot->boidData.separateX = 0;
    robot->boidData.separateY = 0;
    robot->boidData.separateXObs = 0;
    robot->boidData.separateYObs = 0;
    robot->boidData.averageXPos = 0;
    robot->boidData.averageYPos = 0;
    robot->boidData.numNeighbours = 0;

    // Setting stored velocity to the real velocity so it doesn't grow too large in magnitude
    robot->xVel = vels2.xvel;
    robot->yVel = vels2.yvel;

    // Adding to the test data vector
    robot->testingDistances.push_back(CalculateDistance(Pose(robot->stack->xpos, robot->stack->ypos, 0, 0), robot));

    return 0;
}

void ConvoyRobot::TestingStall(ConvoyRobot *robot) {
    // Creating a filestream to the output csv
    std::ofstream dataFile("../testing/boids_follow_classic_results/output.csv", std::ios::app);
    if (!dataFile.is_open()){
        std::cerr << "File failed to open: " << std::strerror(errno) << std::endl;
        return;
    }

    // If the robot has not stalled then time to stall is just 0
    unsigned long result = 0.0;
    if(robot->pos->Stalled()) {
        // Calculating the time this robot took to stall
        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
        long long milliseconds_count = milliseconds.count();
        unsigned long timeNow = static_cast<unsigned long>(milliseconds_count);

        result = (timeNow - robot->startTime);
        result *= timeScale / 2;
        result /= 1000; // in seconds
    }

    // Calculating the average distance
    double result2 = 0.0;
    for(double i : robot->testingDistances) {
        result2 += i;
    }
    result2 = result2 / robot->testingDistances.size();

    if(std::isnan(result2)) {
        dataFile.close();
        return;
    }

    dataFile << robot->id << "," << result2 << "," << result << std::endl;
    dataFile.close();
}

std::pair<double, double> ConvoyRobot::CalculatePosition(double a, Pose pose, double distance) {
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

ConvoyRobot::HVelocities ConvoyRobot::CalculateHolonomic(double linearvel, double turnvel, ConvoyRobot *robot) {
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

ConvoyRobot::NHVelocities ConvoyRobot::CalculateNonHolonomic(double xvel, double yvel, ConvoyRobot *robot) {
    // Declaring a struct to hold velocity values
    NHVelocities vels;

    // Finding magnitude of linear velocity vector

    vels.linearVel = sqrt(pow((xvel), 2.0) + pow((yvel), 2.0));

    // Computing the rotational velocity

    double newDirection = atan2(yvel, xvel);
    double angleDiff = newDirection - robot->GetPose().a;

    vels.rotationalVel = angleDiff / (1.0/timeScale);

    return vels;
}

// Calculating the distance between the current robot and the given pose of another robot
double ConvoyRobot::CalculateDistance(Pose pose, ConvoyRobot *robot) {
    Pose poseThis = robot->pos->GetPose();
    double xDiff = (double)(poseThis.x - pose.x);
    double yDiff = (double)(poseThis.y - pose.y);

    // Pythagoras theorem used to calculate the distance between two points

    double distance = sqrt(abs((xDiff * xDiff) + (yDiff * yDiff)));
    return distance;
}

// Push function to push to the stack and handle keeping only 2 data structures present
void ConvoyRobot::push(double xvel, double yvel, ConvoyRobot *robot) {
    // Removing the reference from the head to make head into second
    robot->stack->second = nullptr;

    // Creating the new head
    struct VipVelocityNode* newhead = newNode(xvel, yvel);

    // Setting old head as second to new head
    newhead->second = robot->stack;

    // Setting class reference to equal to newhead
    robot->stack = newhead;
}

// Created a new node for the stack with the given velocity data
struct ConvoyRobot::VipVelocityNode* ConvoyRobot::newNode(double xpos, double ypos) {
    struct VipVelocityNode* newNode = (struct VipVelocityNode*)malloc(sizeof(struct VipVelocityNode));
    newNode->xpos = xpos;
    newNode->ypos = ypos;
    newNode->second = nullptr;
    return newNode;
}

// Getter method to return the pose of the positional model
Pose ConvoyRobot::GetPose() {
    return this->pos->GetPose();
}