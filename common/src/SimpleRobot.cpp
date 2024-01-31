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

    // Setting the space for the camera array using C++ convention rather than malloc
    this->cameras = new ModelBlobfinder *[8];
    
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

    for(int i = 0; i<camCount; i++) {
        this->cameras[i] = (ModelBlobfinder *) (this->pos->GetChild("blobfinder:" + std::to_string(i)));
        this->cameras[i]->AddCallback(Model::CB_UPDATE, model_callback_t(SensorUpdate), this); // SHOULD ADD EXTRA PARAM FOR INDEX camera[i] for example
        this->cameras[i]->Subscribe();
    }
    
    // Subscribing to callback updates
    this->pos->Subscribe();
    
    // Set the model initial position
    this->pos->SetPose(pose);
}

// Sensor update callback
int SimpleRobot::SensorUpdate(Model *, SimpleRobot* robot) {
    // Getting the array of sensors on the robot

    // Separation
    double close_dx = 0;
    double close_dy = 0;

    // // Alignment
    // double averageXVel = 0;
    // double averageYVel = 0;

    double numNeighbours = 0;

    // Cohesion
    double averageXPos = 0;
    double averageYPos = 0;

    // Non-holonmic velocity value struct
    NHVelocities vels;

    // Array holding the angles
    double angles[8] = {0, 45, 90, 135, 180, -45, -90, -135};

    // Range based for loop necessary as often it will not contain any readings, so presumptions such as blobfinder[0] cause seg faults
    for (int i=0; i<camCount; i++) {
        // Getting the fist blobfinder on the robot

        const std::vector<ModelBlobfinder::Blob> &blobfinder = robot->cameras[i]->GetBlobs();

        for(const ModelBlobfinder::Blob blob : blobfinder){
            int r = blob.color.r * 255;
            int g = blob.color.g * 255;
            int b = blob.color.b * 255;
            int a = blob.color.a * 255;

            int full = (r << 24) | (g << 16) | (b << 8) | a;

            double distance = blob.range;

            // Case for handling obstacles and case for handling other bots
            switch(full) {
                case black:
                    // Need distance to fall lower than the obstacle avoidance distance
                    if (distance < avoidObstructionDistance) {
                        // Get angle of sensor on bot (if negative just add (pi - absolute value) to pi to get positive angle representation)

                        double theta = angles[i];
                        if (theta < 0) theta = (2 * M_PI) - abs(theta);

                        // Get obstacle distance (hypotenuse)

                        double hyp = distance;

                        // Using a composite angle for real world sensor angle

                        Pose pose = robot->GetPose();
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

                        printf("Obstacle Relative: %f, %f Angle: %f Distance: %f\r\n", adj, opp, compositeAngle * (180 / M_PI), hyp);
                        printf("Obstacle Real: %f, %f\r\n", xpos, ypos);

                        close_dx -= pose.x - xpos;
                        close_dy -= pose.y - ypos;
                    }
                    break;
                case blue:
                    // Calculating position of detected bot (same as calculating obstacle position), COMMENTED IN CASE ABOVE ^^
                    double theta = angles[i];
                    if (theta < 0) theta = (2 * M_PI) - abs(theta);
                    double hyp = distance;
                    Pose pose = robot->GetPose();
                    double botAngle = pose.a;
                    if (botAngle < 0) botAngle = (2 * M_PI) - abs(botAngle);
                    double compositeAngle = botAngle + theta;
                    if (compositeAngle > (2 * M_PI)) compositeAngle -= (2 * M_PI);
                    double opp = hyp * sin(compositeAngle); // y
                    double adj = hyp * cos(compositeAngle); // x
                    double xpos = pose.x + adj;
                    double ypos = pose.y + opp;

                    printf("Bot Relative: %f, %f Angle: %f Distance: %f\r\n", adj, opp, compositeAngle * (180 / M_PI), hyp);
                    printf("Bot Real: %f, %f\r\n", xpos, ypos);

                    // If the distance is within the avoidance range of the robot
                    if(distance <= avoidanceDistance) {
                        // Separation
                        close_dx += robot->GetPose().x - xpos;
                        close_dy += robot->GetPose().y - ypos;
                    }

                    // If the distance is within the vision range of the robot but outside avoidance range
                    if(distance <= visionRange) {
                        // Alignment
                        // averageXVel += robot->robots[i].xVel;
                        // averageYVel += robot->robots[i].yVel;
                        
                        numNeighbours += 1;

                        // Cohesion
                        averageXPos += xpos;
                        averageYPos += ypos;
                    }

                    break;
            }
        }
    }

    // printf("close_dx: %f, close_dy: %f\r\n", close_dx, close_dy);

    robot->xVel -= close_dx * avoidObstructionFactor;
    robot->yVel -= close_dy * avoidObstructionFactor;

    // Updating velocity for alignment

    if(numNeighbours > 0) {
        // // Calculating the averages for velocity
        // averageXVel = averageXVel / numNeighbours;
        // averageYVel = averageYVel / numNeighbours;

        // Calculating the averages for position
        averageXPos = averageXPos / numNeighbours;
        averageYPos = averageYPos / numNeighbours;

        // // Alignment
        // robot->xVel += (averageXVel - robot->xVel) * alignmentFactor;
        // robot->yVel += (averageYVel - robot->yVel) * alignmentFactor;

        // Cohesion
        robot->xVel += (averageXPos - robot->GetPose().x) * cohesionFactor;
        robot->yVel += (averageYPos - robot->GetPose().y) * cohesionFactor;
    }

    vels = CalculateNonHolonomic(robot->xVel, robot->yVel, robot);

    // Setting values for non-holonomic system
    robot->pos->SetSpeed(vels.linearVel, 0, vels.rotationalVel);

    return 0;
}

// Position update function for stage (necessary for the bot to actually move)
int SimpleRobot::PositionUpdate(Model *, SimpleRobot* robot) {
    return 0;
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