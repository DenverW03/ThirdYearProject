# A custom positional model for a simulated robot
define custombot position
(   
    # Actual in world size of the model
    size [0.5 0.375 1]

    # Center of rotation offset
    origin [0.125 0 0 0]

    # Draws indicator for where robot is facing
    gui_nose 1

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

    # Non-holonomic movement
    drive "diff"

    # Defining a custom sonar module to sense obstacles
    define customsonar sensor
    (
        size [0.01 0.05 0.01] 
        range [0 4.0]
        fov 15
        samples 1
    )

    # The ranger device holds the above defined sonars
    define customranger ranger
    (
        customsonar(pose [0 0 0 0])
        customsonar(pose [0 0.75 0 180])
        customsonar(pose [1 0.75 0 90])
        customsonar(pose [1 0 0 -90])
    )

    customsonar()
    customranger()
)