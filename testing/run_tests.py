import sys
import subprocess

## Function to build the parameter file
def build_parameters(directory, string):
    with open(directory + '/bot/Parameters.hh', 'w', encoding='utf-8') as param_file:
        param_file.writelines(string)

## Setup stage, takes directory as string and the number of robots
def world_setup(directory, num_robots):
    ## WORLDFILE WRITING
    # Import template world file
    file = open(directory + "/world/template.world")
    # Read file to string buffer
    fileRead = file.read()
    file.close()

    # Adding the VIP agent
    fileRead = fileRead + '\r\nvipbot\r\n(\r\nname "VipRobot"\r\n)'

    # Append some number of robots based on number to buffer string
    for i in range(num_robots):
        fileRead = fileRead + '\r\ncustombot\r\n(\r\nname "ConvoyRobot' + str(i+1) + '"\r\n)'

    # Write buffer string to file
    f = open(directory + "/world/generated.world", "w")
    f.write(fileRead)
    f.close()

    ## MAIN METHOD WRITING
    # Editing the main method to match
    data = ""
    with open(directory + 'main_template.cpp','r',encoding='utf-8') as mainFile:
        data = mainFile.readlines()

    # Edit line 6 to ensure we malloc correct data size
    data[5] = 'ConvoyRobot *robots = (ConvoyRobot *) malloc(' + str(num_robots) + ' * sizeof(ConvoyRobot));\r\n'

    # Edit line 13 to add the VIP robot spawning
    data[12] = "    VipRBT::VipRobot vip = VipRBT::VipRobot((ModelPosition *)world.GetModel(argv[2]), Pose(-8, 8, 0, 0));\r\n"

    # Edit line 13 to add robot spawning lines
    for i in range(num_robots):
        data[12] = data[12] + '    robots[' + str(i) + '] = ConvoyRobot((ModelPosition *)world.GetModel(argv[' + str(i + 3) + ']), Pose::Random(-12, 12, -12, 12));\r\n'

    ## SHELL SCRIPT WRITING
    # Write new file structure to main
    with open(directory + 'main.cpp', 'w', encoding='utf-8') as mainFile:
        mainFile.writelines(data)

    # Editing the template shell script to load the models correctly
    data = ""
    with open('run_template.sh','r',encoding='utf-8') as mainFile:
        data = mainFile.readlines()

    # Edit line 13 to add robot spawning lines
    buffer = './main ./world/generated.world VipRobot'
    for i in range(num_robots):
        buffer = buffer + ' ' + 'ConvoyRobot' + str(i + 1)
    data[5] = buffer

    # Write the new shell script
    with open(directory + 'run.sh', 'w', encoding='utf-8') as mainFile:
        mainFile.writelines(data)

num_robots = int(sys.argv[1])
string = "#define visionRange 10\r\n#define cohesionFactor 0.005\r\n#define avoidanceDistance 2\r\n#define avoidanceFactor 0.1\r\n#define avoidObstructionDistance 3\r\n#define avoidObstructionFactor 0.1\r\n#define vipMinDistance 3\r\n#define vipCohesionMultiplier 0.01\r\n#define vipSeparationMultiplier 0.5\r\n#define vipAlignmentMultiplier 0.02\r\n#define vipCircleRadius 5\r\n#define vipCircleNumPoints 360\r\n#define vipBoundingDistance 7\r\n#define velocityPollingRate 2"
build_parameters("../boids_follow_circle", string)
world_setup("../boids_follow_circle", num_robots)
