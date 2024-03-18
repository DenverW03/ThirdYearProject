import subprocess
import sys

## Setup stage
def world_setup(numRobots):
    # Change world speed
    with open("./world/template.world", 'r', encoding='utf-8') as template:
        file_read = template.readlines()
        file_read[4] = "speedup 1\r\n"
        with open("./world/generated.world", 'w', encoding='utf-8') as generated:
            generated.writelines(file_read)

    # Import template world file
    file = open("./world/generated.world")
    # Read file to string buffer
    fileRead = file.read()
    file.close()

    # Adding the VIP agent
    fileRead = fileRead + '\r\nvipbot\r\n(\r\nname "VipRobot"\r\n)'

    # Append some number of robots based on number to buffer string
    for i in range(numRobots):
        fileRead = fileRead + '\r\ncustombot\r\n(\r\nname "ConvoyRobot' + str(i+1) + '"\r\n)'

    # Write buffer string to file
    f = open("./world/generated.world", "w")
    f.write(fileRead)
    f.close()

    # Editing the main method to match
    data = ""
    with open('main_template_demo.cpp','r',encoding='utf-8') as mainFile:
        data = mainFile.readlines()

    # Add the robot array allocation
    data[12] = "    ConvoyRobot *robots = (ConvoyRobot *) malloc(" + str(numRobots) + " * sizeof(ConvoyRobot));\r\n"

    # Aadd the VIP robot spawning
    data[12] += "    VipRBT::VipRobot vip = VipRBT::VipRobot((ModelPosition *)world.GetModel(argv[2]), Pose(-8, 8, 0, 0));\r\n"

    # Add convoy robot spawning lines
    for i in range(numRobots):
        data[12] = data[12] + '    robots[' + str(i) + '] = ConvoyRobot((ModelPosition *)world.GetModel(argv[' + str(i + 3) + ']), Pose::Random(-12, 12, -12, 12), ' + str(i + 1) + ');\r\n'

    # Write new file structure to main
    with open('main.cpp', 'w', encoding='utf-8') as mainFile:
        mainFile.writelines(data)

    # Editing the template shell script to load the models correctly
    data = ""
    with open('run_template.sh','r',encoding='utf-8') as mainFile:
        data = mainFile.readlines()

    # Edit line 13 to add robot spawning lines
    buffer = './main ./world/generated.world VipRobot'
    for i in range(numRobots):
        buffer = buffer + ' ' + 'ConvoyRobot' + str(i + 1)
    data[5] = buffer

    # Write the new shell script
    with open('run.sh', 'w', encoding='utf-8') as mainFile:
        mainFile.writelines(data)

## Function to build the parameter file
def build_parameters(string):
    with open('./bot/Parameters.hh', 'w', encoding='utf-8') as param_file:
        param_file.writelines(string)

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

## Running stage
def run_simulation(numRobot):
    subprocess.call(["make", "Makefile", "build"])

    try:
        # Run the shell script to run simulation
        subprocess.call(["sh","run.sh"])
    # Ignoring keyboard interrupts when ending simulation because they are annoying :)
    except KeyboardInterrupt:
        sys.exit(0)
    except Exception as e:
        print("An error occurred:", e)


# Use command line to choose number of robots
numRobots = int(sys.argv[1])
string = "#define visionRange 1\r\n#define cohesionFactor 1\r\n#define avoidanceDistance 1\r\n#define avoidanceFactor 1\r\n#define avoidObstructionDistance 1\r\n#define avoidObstructionFactor 1\r\n#define vipMinDistance 1\r\n#define vipMaxDistance 1\r\n#define vipCohesionMultiplier 1\r\n#define vipSeparationMultiplier 1\r\n#define vipAlignmentMultiplier 1\r\n#define velocityPollingRate\r\n#define testing 1\r\n#define timeScale 1"
params = [10, # vision range
          0.005, # cohesion factor
          2, # avoidance distance
          0.1, # avoidance factor
          3, # avoid obstruction distance
          0.1, # avoid obstruction factor
          3, # vip min distance
          6, # vip max distance
          5, # vip cohesion multiplier
          0.01, # vip separation multiplier
          0.01, # vip alignment mulitiplier
          2000, # velocity polling rate
          0, # testing
          1, # timeScale
          ]
final_string = create_param_string(string, params)
build_parameters(final_string)
world_setup(numRobots)
run_simulation(numRobots)