# A custom positional model for a simulated robot

# Defining a custom sonar module to sense obstacles
define customsonar sensor
(
    size [0.01 0.05 0.01]
    range [0 4.0]
    fov 21
    samples 1
    # obstacle_return 0
)

# The ranger device holds the above defined sonars
define customranger ranger
(
    customsonar(pose [0 0 0 0])
    # customsonar(pose [0 0 0 45])
    # customsonar(pose [0 0 0 90])
    # customsonar(pose [0 0 0 135])
    # customsonar(pose [0 0 0 180])
    # customsonar(pose [0 0 0 -45])
    # customsonar(pose [0 0 0 -90])
    # customsonar(pose [0 0 0 -135])
)

# The lowest level positional model for the bot
define custombot_base position
(   
    # Some basic attributes
    color "blue"
    drive "diff"
    gui_nose 1
    obstacle_return 1
    ranger_return 1

    # Non-holonomic movement
    drive "diff"

    # actual size
    size [0.5 0.375 1]

    # the pioneer's center of rotation is offset from its center of area
    origin [0 0 0 0]

    # draw a nose on the robot so we can see which way it points
    gui_nose 1

    # estimated mass in KG
    mass 10.0
)

# Base attributes given shape
define custombot custombot_base
(
    # Adding the sonar array contained in the ranger
    customranger(pose [0 0 0 0])

    # Shape of the robot positional model
    block
    (
        points 4
        point[0] [0 0]
        point[1] [0 0.75]
        point[2] [1 0.75]
        point[3] [1 0]

        z [0 1]
    )
)