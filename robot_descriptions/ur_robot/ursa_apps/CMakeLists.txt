cmake_minimum_required(VERSION 2.8.3)
project(ursa_apps)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    ur_description
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)



