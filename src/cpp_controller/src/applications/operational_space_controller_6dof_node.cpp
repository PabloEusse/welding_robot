#include <tum_ics_ur_robot_lli/Robot/RobotArmConstrained.h>
#include <tum_ics_ur10_controller_tutorial/operational_space_controller_6dof.h>
#include <QApplication>

int main(int argc, char **argv)
{
    QApplication a(argc, argv);

    ros::init(argc, argv, "operational_space_controller_6dof_node", ros::init_options::AnonymousName);

    if (argc < 2)
    {
        ROS_ERROR_STREAM("Usage: " << argv[0] << " <config_file_path>");
        return -1;
    }

    QString configFilePath = argv[1];
    ROS_INFO_STREAM("Config File: " << configFilePath.toStdString().c_str());

    // Create and initialize robot arm
    tum_ics_ur_robot_lli::Robot::RobotArmConstrained robot(configFilePath);
    if (!robot.init())
    {
        ROS_ERROR_STREAM("Failed to initialize robot!");
        return -1;
    }

    // Create controller
    ROS_INFO_STREAM("Creating Operational Space Controller 6DOF");
    tum_ics_ur_robot_lli::RobotControllers::OperationalSpaceController6DOF controller(1.0, "OperationalSpaceCtrl6DOF");

    // Add controller to robot
    ROS_INFO_STREAM("Adding controller to robot");
    if (!robot.add(&controller))
    {
        ROS_ERROR_STREAM("Failed to add controller to robot!");
        return -1;
    }

    // Set initial configurations
    controller.setQHome(robot.qHome());
    controller.setQPark(robot.qPark());

    // Start robot
    ROS_INFO_STREAM("Starting robot");
    robot.start();

    ROS_INFO_STREAM("Spinning...");
    ros::spin();

    ROS_INFO_STREAM("Stopping robot");
    robot.stop();
    ROS_INFO_STREAM("Robot stopped successfully");

    return 0;
}
