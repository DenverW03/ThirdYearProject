import subprocess
import sys

## Setup stage

# Import template world file
file = open("./world/template.world")
# Read file to string buffer
fileRead = file.read()
file.close()

# Use command line to choose number of robots
numRobots = int(sys.argv[1])

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
with open('main_template.cpp','r',encoding='utf-8') as mainFile:
    data = mainFile.readlines()

# Edit line 6 to ensure we malloc correct data size
data[5] = 'ConvoyRobot *robots = (ConvoyRobot *) malloc(' + str(numRobots) + ' * sizeof(ConvoyRobot));\r\n'

# Edit line 13 to add the VIP robot spawning
data[12] = "    VipRBT::VipRobot vip = VipRBT::VipRobot((ModelPosition *)world.GetModel(argv[2]), Pose(-8, 8, 0, 0));\r\n"

# Edit line 13 to add robot spawning lines
for i in range(numRobots):
    data[12] = data[12] + '    robots[' + str(i) + '] = ConvoyRobot((ModelPosition *)world.GetModel(argv[' + str(i + 3) + ']), Pose::Random(-12, 12, -12, 12));\r\n'

# Write new file structure to main
with open('main.cpp', 'w', encoding='utf-8') as mainFile:
    mainFile.writelines(data)


## Running stage

# Running the terminal commands to get the simulation running
subprocess.call(["sh","compile.sh"])

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

try:
    # Run the shell script to run simulation
    subprocess.call(["sh","run.sh"])
# Ignoring keyboard interrupts when ending simulation because they are annoying :)
except KeyboardInterrupt:
    sys.exit(0)
except Exception as e:
    print("An error occurred:", e)