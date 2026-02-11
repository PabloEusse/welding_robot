#!/usr/bin/env python3
"""
Trajectory Publisher Node
=========================

Converts detected target positions into trajectory commands for the UR10 controller.
Supports multiple modes of operation for different use cases.

Modes:
1. CONTINUOUS: Continuously follow detected marker position
2. WAYPOINT: Collect waypoints and execute trajectory
3. TRIGGER: Wait for trigger, then move to current target

The node publishes trajectory targets that the operational space controller
can use to generate smooth trajectories.

Topics Subscribed:
    /trajectory_tracker/target (TrajectoryTarget): Detected target position

Topics Published:
    /trajectory_tracker/trajectory_target (geometry_msgs/PoseStamped): Target pose for controller
    /trajectory_tracker/trajectory_path (nav_msgs/Path): Collected waypoints

Services:
    /trajectory_tracker/start_tracking (std_srvs/Trigger): Start continuous tracking
    /trajectory_tracker/stop_tracking (std_srvs/Trigger): Stop tracking
    /trajectory_tracker/add_waypoint (std_srvs/Trigger): Add current position as waypoint
    /trajectory_tracker/clear_waypoints (std_srvs/Trigger): Clear all waypoints
    /trajectory_tracker/execute_trajectory (std_srvs/Trigger): Execute collected waypoints

Parameters:
    ~mode (str): "continuous", "waypoint", or "trigger" (default: "continuous")
    ~min_move_distance (float): Minimum distance to trigger new target (default: 0.01m)
    ~smoothing_alpha (float): Exponential smoothing factor (default: 0.3)
    ~target_z_offset (float): Z offset from detected position (default: 0.0)
    ~approach_height (float): Height for approach moves (default: 0.05m)
"""

import rospy
import numpy as np
from threading import Lock
from collections import deque

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Float64MultiArray

from trajectory_tracker.msg import TrajectoryTarget
from trajectory_tracker.srv import StartRecording, StartRecordingResponse, StopRecording, StopRecordingResponse, PlaybackTrajectory, PlaybackTrajectoryResponse


class TrajectoryPublisherNode:
    def __init__(self):
        rospy.init_node('trajectory_publisher_node')

        # Parameters
        self.mode = rospy.get_param('~mode', 'continuous')
        self.min_move_distance = rospy.get_param('~min_move_distance', 0.01)
        self.smoothing_alpha = rospy.get_param('~smoothing_alpha', 0.3)
        self.target_z_offset = rospy.get_param('~target_z_offset', 0.0)
        self.approach_height = rospy.get_param('~approach_height', 0.05)
        self.max_velocity = rospy.get_param('~max_velocity', 0.1)  # m/s

        # State
        self.is_tracking = False
        self.current_target = None
        self.smoothed_target = None
        self.last_published_target = None
        self.waypoints = []
        self.target_lock = Lock()

        # Recording state
        self.is_recording = False
        self.recorded_waypoints = []  # Waypoints collected during recording
        self.recording_start_time = None

        # History for smoothing (moving average)
        self.target_history = deque(maxlen=5)

        # Publishers
        self.trajectory_target_pub = rospy.Publisher(
            '/trajectory_tracker/trajectory_target', PoseStamped, queue_size=1)
        self.trajectory_path_pub = rospy.Publisher(
            '/trajectory_tracker/trajectory_path', Path, queue_size=1)

        # Publisher for direct Cartesian target (for cpp controller)
        # Format: [x, y, z, enabled]
        self.cartesian_target_pub = rospy.Publisher(
            '/trajectory_tracker/cartesian_target', Float64MultiArray, queue_size=1)

        # Subscribers
        self.target_sub = rospy.Subscriber(
            '/trajectory_tracker/target', TrajectoryTarget, self.target_callback, queue_size=1)

        # Services
        self.start_tracking_srv = rospy.Service(
            '/trajectory_tracker/start_tracking', Trigger, self.start_tracking_service)
        self.stop_tracking_srv = rospy.Service(
            '/trajectory_tracker/stop_tracking', Trigger, self.stop_tracking_service)
        self.add_waypoint_srv = rospy.Service(
            '/trajectory_tracker/add_waypoint', Trigger, self.add_waypoint_service)
        self.clear_waypoints_srv = rospy.Service(
            '/trajectory_tracker/clear_waypoints', Trigger, self.clear_waypoints_service)
        self.execute_trajectory_srv = rospy.Service(
            '/trajectory_tracker/execute_trajectory', Trigger, self.execute_trajectory_service)

        # Recording services
        self.start_recording_srv = rospy.Service(
            '/trajectory_tracker/start_recording', StartRecording, self.start_recording_service)
        self.stop_recording_srv = rospy.Service(
            '/trajectory_tracker/stop_recording', StopRecording, self.stop_recording_service)
        self.playback_trajectory_srv = rospy.Service(
            '/trajectory_tracker/playback_trajectory', PlaybackTrajectory, self.playback_trajectory_service)

        # Timer for publishing trajectory
        self.publish_rate = rospy.get_param('~publish_rate', 30.0)
        self.publish_timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), self.publish_callback)

        rospy.loginfo(f"Trajectory publisher started in {self.mode.upper()} mode")
        rospy.loginfo("Call /trajectory_tracker/start_tracking to begin")

    def target_callback(self, msg: TrajectoryTarget):
        """Store incoming target position."""
        if not msg.valid:
            return

        with self.target_lock:
            self.current_target = np.array([
                msg.position.x,
                msg.position.y,
                msg.position.z + self.target_z_offset
            ])

            # Add to history for smoothing
            self.target_history.append(self.current_target.copy())

            # Apply exponential smoothing
            if self.smoothed_target is None:
                self.smoothed_target = self.current_target.copy()
            else:
                self.smoothed_target = (
                    self.smoothing_alpha * self.current_target +
                    (1 - self.smoothing_alpha) * self.smoothed_target
                )

            # Auto-record waypoints if recording is active
            if self.is_recording:
                self._add_recorded_waypoint(self.smoothed_target.copy())

    def publish_callback(self, event):
        """Periodically publish trajectory target."""
        if not self.is_tracking:
            # Publish disabled state
            self._publish_cartesian_target(None, enabled=False)
            return

        with self.target_lock:
            if self.smoothed_target is None:
                return

            target = self.smoothed_target.copy()

        # Check minimum movement distance
        if self.last_published_target is not None:
            distance = np.linalg.norm(target - self.last_published_target)
            if distance < self.min_move_distance:
                return

        # Publish trajectory target
        self._publish_pose_stamped(target)
        self._publish_cartesian_target(target, enabled=True)

        self.last_published_target = target.copy()

    def _publish_pose_stamped(self, position):
        """Publish target as PoseStamped message."""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "base_link"
        pose_msg.pose.position.x = position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = position[2]
        # Orientation: pointing down (for pen drawing)
        pose_msg.pose.orientation.x = 1.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 0.0

        self.trajectory_target_pub.publish(pose_msg)

    def _publish_cartesian_target(self, position, enabled=True):
        """Publish target for direct Cartesian control."""
        msg = Float64MultiArray()
        if position is not None and enabled:
            msg.data = [position[0], position[1], position[2], 1.0]  # [x, y, z, enabled]
        else:
            msg.data = [0.0, 0.0, 0.0, 0.0]  # Disabled

        self.cartesian_target_pub.publish(msg)

    def start_tracking_service(self, req):
        """Start continuous tracking."""
        self.is_tracking = True
        rospy.loginfo("Tracking STARTED")
        return TriggerResponse(success=True, message="Tracking started")

    def stop_tracking_service(self, req):
        """Stop tracking."""
        self.is_tracking = False
        self._publish_cartesian_target(None, enabled=False)
        rospy.loginfo("Tracking STOPPED")
        return TriggerResponse(success=True, message="Tracking stopped")

    def add_waypoint_service(self, req):
        """Add current target as waypoint."""
        with self.target_lock:
            if self.smoothed_target is None:
                return TriggerResponse(success=False, message="No valid target")

            waypoint = self.smoothed_target.copy()

        self.waypoints.append(waypoint)
        self._publish_path()

        msg = f"Waypoint {len(self.waypoints)} added at ({waypoint[0]:.3f}, {waypoint[1]:.3f}, {waypoint[2]:.3f})"
        rospy.loginfo(msg)
        return TriggerResponse(success=True, message=msg)

    def clear_waypoints_service(self, req):
        """Clear all waypoints."""
        self.waypoints.clear()
        self._publish_path()
        rospy.loginfo("Waypoints cleared")
        return TriggerResponse(success=True, message="Waypoints cleared")

    def execute_trajectory_service(self, req):
        """Execute collected waypoints as trajectory."""
        if len(self.waypoints) == 0:
            return TriggerResponse(success=False, message="No waypoints to execute")

        rospy.loginfo(f"Executing trajectory with {len(self.waypoints)} waypoints")

        # For now, just publish each waypoint sequentially
        # In a real implementation, you would generate a smooth trajectory
        for i, waypoint in enumerate(self.waypoints):
            if rospy.is_shutdown():
                break

            rospy.loginfo(f"Moving to waypoint {i+1}/{len(self.waypoints)}")
            self._publish_pose_stamped(waypoint)
            self._publish_cartesian_target(waypoint, enabled=True)

            # Wait for robot to reach waypoint (simple delay for now)
            rospy.sleep(2.0)

        self._publish_cartesian_target(None, enabled=False)
        rospy.loginfo("Trajectory execution complete")
        return TriggerResponse(success=True, message=f"Executed {len(self.waypoints)} waypoints")

    def _publish_path(self):
        """Publish waypoints as Path message for visualization."""
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "base_link"

        for waypoint in self.waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.position.z = waypoint[2]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.trajectory_path_pub.publish(path_msg)

    def _add_recorded_waypoint(self, waypoint):
        """
        Add a waypoint to recorded trajectory.
        Only adds if waypoint is far enough from the last recorded point.
        """
        if len(self.recorded_waypoints) == 0:
            self.recorded_waypoints.append(waypoint)
            rospy.loginfo(f"First waypoint recorded at ({waypoint[0]:.3f}, {waypoint[1]:.3f}, {waypoint[2]:.3f})")
            return

        # Check distance from last waypoint
        last_waypoint = self.recorded_waypoints[-1]
        distance = np.linalg.norm(waypoint - last_waypoint)

        if distance >= self.min_move_distance:
            self.recorded_waypoints.append(waypoint)
            rospy.loginfo_throttle(0.5, f"Waypoint {len(self.recorded_waypoints)} recorded")

    def start_recording_service(self, req):
        """Start recording trajectory waypoints."""
        response = StartRecordingResponse()

        if self.is_recording:
            response.success = False
            response.message = "Recording already in progress!"
            return response

        with self.target_lock:
            self.is_recording = True
            self.recorded_waypoints = []
            self.recording_start_time = rospy.Time.now()

        response.success = True
        response.message = "Recording started! Move the pen/marker to draw the trajectory."

        rospy.loginfo("=" * 60)
        rospy.loginfo("TRAJECTORY RECORDING STARTED")
        rospy.loginfo("Move the detected marker/pen to draw the trajectory")
        rospy.loginfo("Call /trajectory_tracker/stop_recording when done")
        rospy.loginfo("=" * 60)

        return response

    def stop_recording_service(self, req):
        """Stop recording and save the trajectory."""
        response = StopRecordingResponse()

        if not self.is_recording:
            response.success = False
            response.num_waypoints = 0
            response.message = "No recording in progress!"
            return response

        with self.target_lock:
            self.is_recording = False
            num_waypoints = len(self.recorded_waypoints)
            recording_duration = (rospy.Time.now() - self.recording_start_time).to_sec()

            # Copy recorded waypoints to main waypoints list for playback
            self.waypoints = [wp.copy() for wp in self.recorded_waypoints]

        response.success = True
        response.num_waypoints = num_waypoints
        response.message = f"Recording stopped! Captured {num_waypoints} waypoints in {recording_duration:.1f}s"

        rospy.loginfo("=" * 60)
        rospy.loginfo(f"TRAJECTORY RECORDING STOPPED")
        rospy.loginfo(f"  Waypoints: {num_waypoints}")
        rospy.loginfo(f"  Duration: {recording_duration:.1f}s")
        rospy.loginfo(f"Call /trajectory_tracker/playback_trajectory to execute")
        rospy.loginfo("=" * 60)

        # Publish path for visualization
        self._publish_path()

        return response

    def playback_trajectory_service(self, req):
        """Playback recorded trajectory (delegates to execute_trajectory)."""
        response = PlaybackTrajectoryResponse()

        if len(self.waypoints) == 0:
            response.success = False
            response.message = "No trajectory recorded! Record a trajectory first."
            return response

        rospy.loginfo(f"Playing back trajectory with {len(self.waypoints)} waypoints...")

        # Use existing execute_trajectory functionality
        trigger_response = self.execute_trajectory_service(None)

        response.success = trigger_response.success
        response.message = trigger_response.message

        return response


def main():
    try:
        node = TrajectoryPublisherNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
