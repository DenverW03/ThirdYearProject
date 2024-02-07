# The VIP bot, simply a larger bot with a pre-planned movement path

# Interaction attributes
define vipbot_base position
(
    # Basic parameters for interaction with environment
    color "red"
    drive "diff"
    gui_nose 1
    obstacle_return 1
    ranger_return 1
    blob_return 1
    fiducial_return 1

    localization "gps"         	  
    localization_origin [0 0 0 0]
)

# Physical attributes
define vipbot_base_final custombot_base
(
    size [0.44 0.38 0.22]

    origin [-0.04 0 0 0]

    gui_nose 1

    mass 10.0 

    drive "diff"
)

# Final model, defines shape
define vipbot custombot_base_final
(
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