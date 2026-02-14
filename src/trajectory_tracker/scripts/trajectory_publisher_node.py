#!/usr/bin/env python3
"""
Trajectory Publisher Node (Simplified)
=======================================

Simple node that receives extracted trajectory path and republishes it
in formats suitable for robot controllers.

Topics Subscribed:
    /trajectory_tracker/extracted_path (nav_msgs/Path): Extracted trajectory from line detection

Topics Published:
    /trajectory_tracker/trajectory_path (nav_msgs/Path): Same path (for compatibility)
    /trajectory_tracker/cartesian_target (std_msgs/Float64MultiArray): First point [x, y, z, enabled]

This simplified version removes all complex modes (continuous, waypoint, trigger, recording)
and focuses solely on republishing the extracted trajectory for robot control.
"""

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray


class TrajectoryPublisherNode:
    def __init__(self):
        rospy.init_node('trajectory_publisher_node')

        # Shutdown flag
        self.is_shutdown = False

        # Publishers
        self.trajectory_path_pub = rospy.Publisher(
            '/trajectory_tracker/trajectory_path', Path, queue_size=1)
        self.cartesian_target_pub = rospy.Publisher(
            '/trajectory_tracker/cartesian_target', Float64MultiArray, queue_size=1)

        # Subscribers
        self.path_sub = rospy.Subscriber(
            '/trajectory_tracker/extracted_path', Path, self.path_callback, queue_size=1)

        # Register shutdown hook
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("Trajectory publisher started (simplified mode)")
        rospy.loginfo("Subscribing to /trajectory_tracker/extracted_path")

    def path_callback(self, msg: Path):
        """
        Receive extracted path and republish for robot control.
        """
        if self.is_shutdown:
            return

        if len(msg.poses) == 0:
            rospy.logwarn("Received empty path")
            return

        # Republish path as-is
        self.trajectory_path_pub.publish(msg)

        # Publish first point as cartesian target for direct control
        first_pose = msg.poses[0]
        cartesian_msg = Float64MultiArray()
        cartesian_msg.data = [
            first_pose.pose.position.x,
            first_pose.pose.position.y,
            first_pose.pose.position.z,
            1.0  # Enabled flag
        ]
        self.cartesian_target_pub.publish(cartesian_msg)

        rospy.loginfo(f"Published trajectory with {len(msg.poses)} points")

    def shutdown(self):
        """Clean up on shutdown."""
        rospy.loginfo("Shutting down trajectory publisher...")
        self.is_shutdown = True

        # Publish disabled state
        disabled_msg = Float64MultiArray()
        disabled_msg.data = [0.0, 0.0, 0.0, 0.0]
        self.cartesian_target_pub.publish(disabled_msg)

        rospy.loginfo("Trajectory publisher shutdown complete")


def main():
    try:
        node = TrajectoryPublisherNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
