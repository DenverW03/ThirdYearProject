import subprocess
import sys

# Import template world file
file = open("./template.world")
# Read file to string buffer
fileRead = file.read()
file.close()
# Take input for number of robots to add
# numRobots = int(input("Number of robots in simulation: "))
numRobots = int(sys.argv[1])
# Append some number of robots based on number to buffer string
for i in range(numRobots):
    fileRead = fileRead + '\r\npioneer2dx\r\n(\r\nname "MySimpleRobot' + str(i+1) + '"\r\npose [ -6.946 -6.947 0 45.000 ]\r\nsicklaser( pose [ 0 0 0 0 ] )\r\nlocalization "gps"\r\nlocalization_origin [ 0 0 0 0 ]\r\ntrail_length 400\r\n)'
# Write buffer string to file
f = open("../common/world/myworld.world", "w")
f.write(fileRead)
f.close()

# Editing the main method to match
with open('main-template.cpp','r',encoding='utf-8') as mainFile:
    data = mainFile.readlines()
# Edit line 13 to add robot spawning lines
for i in range(numRobots):
    data[12] = data[12] + '\r\n    robots[' + str(i) + '] = SimpleRobot((ModelPosition *)world.GetModel(argv[' + str(i + 2) + ']), Pose::Random(-4, 4, -4, 4), robots);'
# Write new file structure to main
with open('main.cpp', 'w', encoding='utf-8') as mainFile:
    mainFile.writelines(data)

# Running the terminal commands to get the simulation running
subprocess.call(["sh","compile.sh"])
# Editing the main method to match
with open('run.sh','r',encoding='utf-8') as mainFile:
    data = mainFile.readlines()
# Edit line 13 to add robot spawning lines
buffer = './main ../common/world/myworld.world'
for i in range(numRobots):
    buffer = buffer + ' ' + 'MySimpleRobot' + str(i + 1)
data[3] = buffer
# Write the new shell script
with open('run.sh', 'w', encoding='utf-8') as mainFile:
    mainFile.writelines(data)
# Run the shell script to run simulation
subprocess.call(["sh","run.sh"])