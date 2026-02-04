# Force Toque Sensor Setup and ROS Initialization Guide

## Setup Steps

1. Insert plastic welding gun into gripper as described in `gripper_setup_sensor_setup_guide.md`

2. Open web browser and navigate to `http://192.168.1.1`

3. Reset force/torque data (calibration):
   - Go to **Snapshot**
   - Click **Bias**
   - Click **Reset Peaks**

## Terminal Setup

Open 4 terminals and run the following commands in each:

### Terminal I - ROS Core
```bash
catkin build
source devel/setup.bash
rosclean purge -y
roscore
```

### Terminal II - Robot Manager
```bash
source devel/setup.bash
roslaunch tum_ics_ur_robot_manager robot_script_manager_ur10.launch
```

### Terminal III - Sensor Publisher
```bash
source devel/setup.bash
roslaunch tum_ics_schunk_netbox sensor_publisher.launch
```

### Terminal IV - Monitor Raw Data (read out force and torque in ROS)
```bash
source devel/setup.bash
rostopic echo /schunk_netbox/raw
```
