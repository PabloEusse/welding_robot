import numpy as np
import rospy

from msbrdm_libs.spline import LinearSplineTrajectory
from ur10_model.ur10_model_3dof import UR10Model3DOF
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class UR10KinematicController3DOF:
    def __init__(self, model: UR10Model3DOF, q_init: np.ndarray = np.zeros(3)) -> None:
        """Kinematic controller for the UR10 robot with 3 DOF

        Args:
            model (UR10Model3DOF): robot model
            q_init (np.ndarray, optional): initial joint positions. Defaults to np.zeros(3).
        """
        self.model = model
        self.q = q_init

        # TODO: calculate the initial end-effector position in {w}
        self.x = self.model.T_ee_w_fn(self.q[0], self.q[1], self.q[2])[:3, 3]
        #rospy.init_node("path_publisher")
        self.time = 0
        #######################################################################
        # Initializations
        # end-effector goal position in {w}
        self.x_goal = np.zeros(3)
        # trajectory duration (default: 10s)
        self.duration = 10.0
        # singularity threshold
        self.w_threshold = 0.0

        self.trajectory = None
        self.start_time = None

        #######################################################################
        # (Optional) TODO: Publish the trajectory as nav_msgs/Path for visualization
        self.path_pub = rospy.Publisher("/trajectory_path", Path, queue_size=10)

    def set_goal_abs(self, x_goal):
        """Set goal position with absolute coordinates in {w}

        Args:
            x_goal (_type_): absolute goal position in {w}
        """
        # TODO: set member variable self.x_goal
        self.x_goal = x_goal

    def set_goal_rel(self, x_goal):
        """Set goal position with relative coordinates in {w}

        Args:
            x_goal (_type_): relative goal position in {w}
        """
        # TODO: set member variable self.x_goal
        self.x_goal = x_goal
    
    def set_duration(self, duration: float):
        """Set the duration of the trajectory

        Args:
            duration (float): duration
        """
        # TODO: set member variable self.duration
        self.duration = duration

    def set_w_threshold(self, threshold: float):
        """Set the threshold for manipulability index

        Args:
            threshold (float): threshold value
        """
        # TODO: set member variable self.w_threshold
        self.w_threshold = threshold

    def start(self):
        # TODO: generate the trajectory with the initial start and goal position
        # as the type of LinearSplineTrajectory
        self.trajectory = LinearSplineTrajectory(self.x, self.x_goal, self.duration)

        # save the start time
        self.start_time = rospy.Time.now()

        # Optional: publish the trajectory for visualization

    def publish_path(self, trajectory: LinearSplineTrajectory):
        a0, a1, a2, a3, a4, a5, duration = trajectory.a0, trajectory.a1, trajectory.a2, trajectory.a3, trajectory.a4, trajectory.a5, trajectory.duration
        path = Path()
        path.header.frame_id = "world" 
        for t in range(duration + 1):
            pose = PoseStamped()
            step = a0 + a1 * t + a2 * t**2 + a3 * t**3  + a4 * t**4 + a5 * t**5
            pose.header.frame_id = "world"
            pose.pose.position.x = step[0]
            pose.pose.position.y = step[1]
            pose.pose.position.z = step[2]
            pose.pose.orientation.w = 1.0  # no rotation

            path.poses.append(pose)
        self.path_pub.publish(path)

    def update(self, joint_states: np.ndarray,
               time: rospy.Time, dt: rospy.Duration) -> np.ndarray:
        """ Update the controller

        Args:
            joint_states (np.ndarray): current joint states from robot feedback [q, qdot, qddot] (3 x n)
            time (rospy.Time): current ros time
            dt (rospy.Duration): time interval

        Returns:~
            np.ndarray: controller output u
        """

        # retrieve joint positions and velocities
        self.q = joint_states[0, :]
        self.qdot = joint_states[1, :]

        ########################################################################
        # TODO: Implement the kinematic controller

        # end-effector position
        ee_pos = self.model.T_ee_w_fn(self.q[0], self.q[1], self.q[2])[:3, 3]
        # end-effector jacobian
        J_ee = self.model.J_ee_w_fn(self.q[0], self.q[1], self.q[2])
        # extract jacobian for linear velocity only
        J_ee_v = J_ee[3:, :]
        # compute the manipulability index
        w = np.sqrt(np.linalg.det(J_ee_v @ J_ee_v.T))
        x_des, v_des, a_des = self.trajectory.update(self.time)
        
        self.time += dt.to_sec()
        # get desired cartesian states from the trajectory
        v_d = x_des - ee_pos
        # implement the kinematic controller
        self.publish_path(self.trajectory)
        
        return np.linalg.pinv(J_ee_v) @ v_d if w > self.w_threshold else np.zeros(3)