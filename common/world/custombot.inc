# A custom bot with sonar and color detection

# Stats based off the HC-SR04 sonar module
define HC-SR04 sensor
(
  size [0.01 0.05 0.01] 
  range [0 4.0]
  fov 21
  samples 1
  # obstacle_return 1
)

# A collection of HC-SR04 sensors
define sonar_array ranger
(
  # Sonar sensor array on bot
    HC-SR04(pose [0 0 0 0])
    HC-SR04(pose [0 0 0 45])
    HC-SR04(pose [0 0 0 90])
    HC-SR04(pose [0 0 0 135])
    HC-SR04(pose [0 0 0 180])
    HC-SR04(pose [0 0 0 -45])
    HC-SR04(pose [0 0 0 -90])
    HC-SR04(pose [0 0 0 -135])
)

# Custom bot base positional model
define custombot_base position
(
  # Basic parameters for interaction with environment
  color "blue"
  drive "diff"
  gui_nose 1
  obstacle_return 1
  ranger_return 1
  blob_return 1
  fiducial_return 1

  localization "gps"         	  
  localization_origin [0 0 0 0]

  # [ xmin xmax ymin ymax zmin zmax amin amax ]				
  velocity_bounds [-0.5 0.5 0 0 0 0 -90.0 90.0]
  acceleration_bounds [-0.5 0.5 0 0 0 0 -90 90.0]
)



define custombot_base_no_sonar custombot_base
(
  # actual size
  size [0.44 0.38 0.22] # sizes from MobileRobots' web site

  # the pioneer's center of rotation is offset from its center of area
  origin [-0.04 0 0 0]

  # draw a nose on the robot so we can see which way it points
  gui_nose 1

  # estimated mass in KG
  mass 10.0 
  # differential steering model
  drive "diff"
 )


define custombot_base_final custombot_base_no_sonar
(
  # use the sonar array defined above with a small vertical offset to
  # drop the sensors into the robot body
  sonar_array( pose [0 0 -0.03 0] )
)

define custombot custombot_base_final
(
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