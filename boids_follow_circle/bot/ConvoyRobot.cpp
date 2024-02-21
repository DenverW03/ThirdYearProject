#include "ConvoyRobot.hh"
#include "CircleVisualizer.hh"
#include <stage.hh>
#include <cmath>
#include <random>
#include <thread>

using namespace Stg;
using namespace ConvoyRBT;

// Default constructor for init
ConvoyRobot::ConvoyRobot(){
    // No setup required in implicit constructor
};

// Constructor for the class with appropriate setup
ConvoyRobot::ConvoyRobot(ModelPosition *modelPos, Pose pose) {
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
    this->boidData.closeDx = 0;
    this->boidData.closeDy = 0;
    this->boidData.closeDxObs = 0;
    this->boidData.closeDyObs = 0;
    this->boidData.closeDxVip = 0;
    this->boidData.closeDyVip = 0;
    this->boidData.averageXPos = 0;
    this->boidData.averageYPos = 0;
    this->boidData.averageXVel = 0;
    this->boidData.averageYVel = 0;
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

    this->lastSysTime = std::time(nullptr);
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

        // Case for handling obstacles and case for handling other bots
        switch(full) {
            case black: { // OBSTACLES
                // Need distance to fall equal to or lower than the obstacle avoidance distance
                if (distance > avoidObstructionDistance) break;

                // Add this blob to the close stacks
                Pose pose = robot->GetPose();
                auto position = CalculatePosition(robot->angles[data->num], pose, distance);
                robot->boidData.closeDxObs -= pose.x - position.first;
                robot->boidData.closeDyObs -= pose.y - position.second;
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
                    robot->boidData.closeDx += pose.x - position.first;
                    robot->boidData.closeDy += pose.y - position.second;
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
                // For red VIP going to have the same vision range as for fellow convoy bots
                if(distance > visionRange) break;

                // Getting the time difference
                unsigned long timeDiff = std::time(nullptr) - robot->lastSysTime;

                Pose pose = robot->GetPose();
                auto position = CalculatePosition(robot->angles[data->num], pose, distance);

                // Alignment, updating the vip Velocities
                double vipEffectX = position.first;
                double vipEffectY = position.second;

                // Separation from VIP
                if(distance <= vipMinDistance) {
                    // robot->boidData.numNeighbours += 1;

                    robot->boidData.closeDxVip += pose.x - position.first;
                    robot->boidData.closeDyVip += pose.y - position.second;
                }
                // else if(distance <= vipCircleRadius) {
                //     double angle = robot->angles[data->num];
                //     if(angle < 0) angle += 180;
                //     else if(angle >= 0) angle -= 180;

                //     auto positionBounding = CalculatePosition(angle, pose, vipBoundingDistance - distance);

                //     robot->boidData.closeDxVip += pose.x - positionBounding.first;
                //     robot->boidData.closeDyVip += pose.y - positionBounding.second;

                //     robot->boidData.numNeighbours += 1;
                // }

                // If it has been over 5 seconds
                if(timeDiff > vipVelPollingRate) {
                    push(vipEffectX, vipEffectY, robot);
                    robot->lastSysTime = std::time(nullptr);
                }
                break;
            }
        }
    }
    return 0;
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

// Returns a vector of paired x,y points that are points on a circle of the radius around the position given as a pose
std::vector<std::pair<double, double>> ConvoyRobot::GeneratePoints(Pose pose, double radius) {
    std::vector<std::pair<double, double>> points;

    // Generating the points and pushing them to the array
    for(int i=0; i<vipCircleNumPoints; i++) {
        // Determining the angle to generate the point at (radians)
        double angle = ((2 * M_PI) / vipCircleNumPoints) * i;

        // Using circle parametric equations
        double x = pose.x + (radius * cos(angle));
        double y = pose.y + (radius * sin(angle));

        points.push_back(std::make_pair(x, y));
    }

    return points;
}

// Position update function for stage (necessary for the bot to actually move)
int ConvoyRobot::PositionUpdate(Model *, ConvoyRobot* robot) {
    // Cohesion for staying as close to "the circle" as possible
    // The circle is a concept of a "belt" around the VIP, much like a planet
    // This belt has a large weighting for attraction towards it

    if(robot->stack->second != nullptr){
        // Getting the last recorded robot positions
        double lastx = robot->stack->xpos;
        double lasty = robot->stack->ypos;

        // Generating a vector of the points
        std::vector<std::pair<double, double>> vecPoint = GeneratePoints(Pose(lastx, lasty, 0, 0), vipCircleRadius);

        // Initialising closest points pair to ridiculously large size so first pair will always be closer
        std::pair<double, double> closest = std::make_pair(10000, 10000);

        // Looping through the points to find the closest one
        for(std::pair<double, double> points : vecPoint) {
            Pose pose1 = Pose(points.first, points.second, 0, 0);
            Pose pose2 = Pose(closest.first, closest.second, 0, 0);
            if(CalculateDistance(pose1, robot) < CalculateDistance(pose2, robot)) closest = points;
        }

        // Add the position as a large weight
        // robot->boidData.averageXPos += closest.first * vipCohesionMultiplier;
        // robot->boidData.averageYPos += closest.second * vipCohesionMultiplier;
        // robot->boidData.numNeighbours += vipCohesionMultiplier;

        robot->xVel += (closest.first - robot->GetPose().x) * vipCohesionMultiplier;
        robot->yVel += (closest.second - robot->GetPose().y) * vipCohesionMultiplier;
    }

    // AVOIDANCE

    // For other bots
    robot->xVel += robot->boidData.closeDx * avoidanceFactor;
    robot->yVel += robot->boidData.closeDy * avoidanceFactor;

    // For obstacles
    robot->xVel -= robot->boidData.closeDxObs * avoidObstructionFactor;
    robot->yVel -= robot->boidData.closeDyObs * avoidObstructionFactor;

    // For the VIP
    robot->xVel += robot->boidData.closeDxVip * vipSeparationMultiplier;
    robot->yVel += robot->boidData.closeDyVip * vipSeparationMultiplier;

    // Cohesion and Alignment BETWEEN CONVOT BOTS

    if(robot->boidData.numNeighbours > 0) {
        // Calculating the averages for position
        robot->boidData.averageXPos = robot->boidData.averageXPos / robot->boidData.numNeighbours;
        robot->boidData.averageYPos = robot->boidData.averageYPos / robot->boidData.numNeighbours;

        // Cohesion
        robot->xVel += (robot->boidData.averageXPos - robot->GetPose().x) * cohesionFactor;
        robot->yVel += (robot->boidData.averageYPos - robot->GetPose().y) * cohesionFactor;

        // Alignment of convoy bot with VIP bot

        if(robot->stack->second != nullptr){
            double dx = robot->stack->xpos - robot->stack->second->xpos;
            double dy = robot->stack->ypos - robot->stack->second->ypos;

            robot->xVel += (robot->xVel - dx) * vipAlignmentMultiplier;
            robot->yVel += (robot->yVel - dx) * vipAlignmentMultiplier;
        }
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
    robot->boidData.closeDxVip = 0;
    robot->boidData.closeDyVip = 0;
    robot->boidData.averageXPos = 0;
    robot->boidData.averageYPos = 0;
    robot->boidData.averageXVel = 0;
    robot->boidData.averageYVel = 0;
    robot->boidData.numNeighbours = 0;

    // Setting stored velocity to the real velocity so it doesn't grow too large in magnitude
    robot->xVel = vels2.xvel;
    robot->yVel = vels2.yvel;
    return 0;
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

    vels.rotationalVel = angleDiff / (1.0/60.0);

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