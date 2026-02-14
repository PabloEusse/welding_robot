cmake_minimum_required(VERSION 3.0.2)
project(trajectory_tracker)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  tf
  message_generation
)

# Generate messages
add_message_files(
  FILES
  TrajectoryTarget.msg
  MarkerArray.msg
  WorkspaceCalibration.msg
)

# Generate services
add_service_files(
  FILES
  GetTargetPosition.srv
  StartRecording.srv
  StopRecording.srv
  PlaybackTrajectory.srv
  CalibrateWorkspace.srv
  ExtractLine.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy roscpp std_msgs sensor_msgs geometry_msgs cv_bridge image_transport tf message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/camera_node.py
  scripts/trajectory_publisher_node.py
  scripts/line_extractor_node.py
  scripts/workspace_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
