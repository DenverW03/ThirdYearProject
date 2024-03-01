import sys
import subprocess
import os

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
    with open(directory + 'main_template_testing.cpp','r',encoding='utf-8') as mainFile:
        data = mainFile.readlines()

    # Replace the number of robots definition
    data[10] = "#define numRobots " + str(num_robots) + "\r\n"

    # Edit line 24 to add robot spawning lines
    for i in range(num_robots):
        data[24] = data[24] + '    robots[' + str(i) + '] = ConvoyRobot((ModelPosition *)world.GetModel(argv[' + str(i + 3) + ']), Pose::Random(-12, 12, -12, 12));\r\n'

    # Write new file structure to main
    with open(directory + 'main.cpp', 'w', encoding='utf-8') as mainFile:
        mainFile.writelines(data)

    ## SHELL SCRIPT WRITING
    # Editing the template shell script to load the models correctly
    data = ""
    with open(directory + 'run_template.sh','r',encoding='utf-8') as mainFile:
        data = mainFile.readlines()

    # Edit line 13 to add robot spawning lines
    buffer = './main ./world/generated.world VipRobot'
    for i in range(num_robots):
        buffer = buffer + ' ' + 'ConvoyRobot' + str(i + 1)
    data[5] = buffer

    # Write the new shell script
    with open(directory + 'run.sh', 'w', encoding='utf-8') as mainFile:
        mainFile.writelines(data)

## Running the simulation
def run_simulation(directory):
    # Compiling the code
    os.chdir(directory)
    subprocess.call(["make", "Makefile", "build"])

    try:
        # Run the shell script to run simulation
        subprocess.call(["sh","run.sh"])
    # Ignoring keyboard interrupts when ending simulation because they are annoying :)
    except KeyboardInterrupt:
        sys.exit(0)
    except Exception as e:
        print("An error occurred:", e)

# Creates a param string from the param template string given
def create_param_string(string, params):
    # Building the string with the parameters passed into function inserted
    final_string = ""
    counter = 0
    for line in string.split("\r\n"):
        temp = line.split(" ")
        new_line = temp[0] + " " + temp[1] + " " + str(params[counter]) + "\r\n"
        final_string += new_line
        counter += 1
    return final_string

## Running a parameter set
def test_parameter_set(params):
    num_robots = int(sys.argv[1])

    # Outlining the string with all parameters replaced with 1 for editing string next
    string = "#define visionRange 1\r\n#define cohesionFactor 1\r\n#define avoidanceDistance 1\r\n#define avoidanceFactor 1\r\n#define avoidObstructionDistance 1\r\n#define avoidObstructionFactor 1\r\n#define vipMinDistance 1\r\n#define vipCohesionMultiplier 1\r\n#define vipSeparationMultiplier 1\r\n#define vipAlignmentMultiplier 1\r\n#define vipCircleRadius 1\r\n#define vipCircleNumPoints 1\r\n#define vipBoundingDistance 1\r\n#define velocityPollingRate 1\r\n#define testing 0\r\n#define timeScale 1"
    final_string = create_param_string(string, params)

    this_path = "../testing/" # The path of this script
    directory = "../boids_follow_circle/" # The path of the current algorithm directory

    build_parameters(directory, final_string)
    world_setup(directory, num_robots)
    run_simulation(directory)

    os.chdir(this_path)

def test_all_sets(sets):
    for params in sets:
        test_parameter_set(params)

params = [
    [10, 0.005, 2, 0.1, 3, 0.1, 3, 0.01, 0.5, 0.02, 5, 360, 7, 2, 1, 60], # First parameter set
    [10, 0.005, 2, 0.1, 3, 0.2, 3, 0.01, 0.5, 0.02, 5, 360, 7, 2, 1, 60]
    ]
test_all_sets(params)