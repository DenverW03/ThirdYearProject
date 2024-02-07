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
    this->cameras = new ModelBlobfinder * [camCount];
    
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
    printf("ID: %d\r\n", data->num);

    // Getting the blobs found in the blobfinder model
    const std::vector<ModelBlobfinder::Blob> &blobs = blobfinder->GetBlobs();

    printf("number of blobs: %d\r\n", blobs.size());

    // Obstacle avoidances
    double closeDxObs = 0;
    double closeDyObs = 0;

    // Separation
    double closeDx = 0;
    double closeDy = 0;

    // // Alignment
    // double averageXVel = 0;
    // double averageYVel = 0;

    double numNeighbours = 0;

    // Cohesion
    double averageXPos = 0;
    double averageYPos = 0;

    // Range based for loop necessary as often it will not contain any readings, so presumptions such as blobfinder[0] cause seg faults
    // for (int i=0; i<camCount; i++) {
        // Getting the fist blobfinder on the robot

        // const std::vector<ModelBlobfinder::Blob> &blobfinder = robot->cameras[i]->GetBlobs();

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
                    // Need distance to fall equal to or lower than the obstacle avoidance distance
                    if (distance > avoidObstructionDistance) break;

                    // NEED TO ENSURE THE CORRECT DEG OR RAD FOR EACH ANGLE, I THOUGHT THAT THESE WERE I RAD BUT THEY ARE IN FACT IN DEGREES

                    // Get angle of sensor on bot (if negative just add (pi - absolute value) to pi to get positive angle representation)
                    double theta = robot->angles[0] * (M_PI / 180);
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

                    // printf("Camera Angle: %f Bot Angle: %f\r\n", theta, botAngle);
                    // printf("Obstacle Relative: %f, %f Angle: %f Distance: %f\r\n", adj, opp, compositeAngle, hyp);
                    // printf("Obstacle Real: %f, %f\r\n", xpos, ypos);

                    closeDxObs -= pose.x - xpos;
                    closeDyObs -= pose.y - ypos;
                    break;
                }
                case blue: {
                    // Guard clause to avoid wasting compute
                    if(distance > visionRange) break;

                    // Calculating position of detected bot (same as calculating obstacle position), COMMENTED IN CASE ABOVE ^^
                    double theta = robot->angles[0] * (M_PI / 180);
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

                    // printf("Bot Relative: %f, %f Angle: %f Distance: %f\r\n", adj, opp, compositeAngle * (180 / M_PI), hyp);
                    // printf("Bot Real: %f, %f\r\n", xpos, ypos);

                    // If the distance is within the avoidance range of the robot

                    // Separation (prior guard clause already confirmed bot to be within avoidance distance)
                    if(distance <= avoidanceDistance) {
                        closeDx += robot->GetPose().x - xpos;
                        closeDy += robot->GetPose().y - ypos;
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
    // }
    // Avoidance

    // For other bots
    robot->xVel += closeDx * avoidanceFactor;
    robot->yVel += closeDy * avoidanceFactor;

    // For obstacles
    robot->xVel -= closeDxObs * avoidObstructionFactor;
    robot->yVel -= closeDyObs * avoidObstructionFactor;

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

    // Setting the new velocity of the robot

    // NHVelocities nonHolonomic = CalculateNonHolonomic(robot->xVel, robot->yVel, robot);
    // robot->pos->SetSpeed(nonHolonomic.linearVel, 0, nonHolonomic.rotationalVel);

    // Setting the stored velocity to the real simulated value

    HVelocities vels2 = CalculateHolonomic(robot->pos->GetVelocity().x, robot->pos->GetVelocity().a, robot);

    std::cout << "--------------------------------\r\n";
    printf("Obstacles - closeDx: %f closeDy: %f\r\n", closeDxObs, closeDyObs);
    std::cout << "--------------------------------\r\n";
    // printf("Calc Polar: %f %f\r\n", vels.linearVel, vels.rotationalVel);
    printf("Real Polar: %f %f\r\n", robot->pos->GetVelocity().x, robot->pos->GetVelocity().a);
    printf("Calc Cartesian: %f %f\r\n", vels2.xvel, vels2.yvel);
    printf("Class velocity: %f %f\r\n", robot->xVel, robot->yVel);
    std::cout << "--------------------------------\r\n";

    // robot->xVel = vels2.xvel;
    // robot->yVel = vels2.yvel;
    // printf("Amended class velocity: %f %f\r\n", robot->xVel, robot->yVel);

    // std::cout << "--------------------------------\r\n";

    // printf("Bots - closeDx: %f closeDy: %f\r\n", closeDx, closeDy);
    // printf("Obstacles - closeDx: %f closeDy: %f\r\n", closeDxObs, closeDyObs);
    // printf("Real velocity: Linear %f Rotational %f\r\n", robot->pos->GetVelocity().x, robot->pos->GetVelocity().a);

    return 0;
}

// Position update function for stage (necessary for the bot to actually move)
int SimpleRobot::PositionUpdate(Model *, SimpleRobot* robot) {
    NHVelocities nonHolonomic = CalculateNonHolonomic(robot->xVel, robot->yVel, robot);
    robot->pos->SetSpeed(nonHolonomic.linearVel, 0, nonHolonomic.rotationalVel);

    printf("position update called\r\n");
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