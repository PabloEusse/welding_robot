cmake_minimum_required(VERSION 3.0.2)
project(weld_line_vision)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  image_transport
  cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(weld_line_vision_node src/weld_line_vision_node.cpp)
target_link_libraries(weld_line_vision_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
