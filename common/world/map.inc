define floorplan model
(
  color "black"

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
  blob_return 1
)