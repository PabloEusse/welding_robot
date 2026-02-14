#!/bin/bash
# Trajectory Tracker Demo Launcher
# 한 번에 모든 시스템을 실행합니다

echo "=================================================="
echo "  Trajectory Tracker Demo Starting..."
echo "=================================================="

# Source ROS environment
source /opt/ros/noetic/setup.bash
source /home/msbrdm_student/welding_robot/devel/setup.bash

# Kill any existing roscore
killall -9 roscore rosmaster 2>/dev/null
sleep 1

# Launch the demo
echo ""
echo "Launching trajectory tracker system..."
echo ""
roslaunch trajectory_tracker demo.launch

# Note: RViz is disabled due to OpenGL issues
# To view visualization, use: rqt_image_view /trajectory_tracker/camera/image_raw
