import subprocess
import sys

# Import template world file
file = open("./template.world")
# Read file to string buffer
fileRead = file.read()
file.close()

# Use command line to choose number of robots
numRobots = int(sys.argv[1])
# Append some number of robots based on number to buffer string
for i in range(numRobots):
    fileRead = fileRead + '\r\ncustombot\r\n(\r\nname "MySimpleRobot' + str(i+1) + '"\r\n)'
# Write buffer string to file
f = open("../common/world/myworld.world", "w")
f.write(fileRead)
f.close()

# Editing the main method to match
data = ""
with open('main_template.cpp','r',encoding='utf-8') as mainFile:
    data = mainFile.readlines()
# Edit line 6 to ensure we malloc correct data size
data[5] = 'SimpleRobot *robots = (SimpleRobot *) malloc(' + str(numRobots) + ' * sizeof(SimpleRobot));\r\n'
# Edit line 13 to add robot spawning lines
for i in range(numRobots):
    data[12] = data[12] + '    robots[' + str(i) + '] = SimpleRobot((ModelPosition *)world.GetModel(argv[' + str(i + 2) + ']), Pose::Random(-12, 12, -12, 12), robots, ' + str(numRobots) + ');\r\n'
# Write new file structure to main
with open('main.cpp', 'w', encoding='utf-8') as mainFile:
    mainFile.writelines(data)

# Running the terminal commands to get the simulation running
subprocess.call(["sh","compile.sh"])
# Editing the template shell script to load the models correctly
data = ""
with open('run_template.sh','r',encoding='utf-8') as mainFile:
    data = mainFile.readlines()
# Edit line 13 to add robot spawning lines
buffer = './main ../common/world/myworld.world'
for i in range(numRobots):
    buffer = buffer + ' ' + 'MySimpleRobot' + str(i + 1)
data[5] = buffer
# Write the new shell script
with open('run.sh', 'w', encoding='utf-8') as mainFile:
    mainFile.writelines(data)

# Run the shell script to run simulation
subprocess.call(["sh","run.sh"])