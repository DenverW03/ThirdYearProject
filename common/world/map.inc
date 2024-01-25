# map.inc - useful setup for a floorplan bitmap
# Authors: Richard Vaughan
# $Id$

define floorplan model
(
  color "gray30"

  # Bounding box
  boundary 1

  # Useful attributes for testing
  gui_nose 0
  gui_grid 0
  gui_move 0
  gui_outline 0

  # Robot and sensor interaction parameters
  gripper_return 1
  fiducial_return 1
  ranger_return 1
  obstacle_return 1
)

define zone model
(
  color	"orange"
  size [4 4 0.01]

  gui_nose 0
  gui_grid 0
  gui_move 0
  gui_outline 0

  # sensible to collision and range sensors
  obstacle_return 1
  
  # visible to range sensors
  ranger_return 1
)
