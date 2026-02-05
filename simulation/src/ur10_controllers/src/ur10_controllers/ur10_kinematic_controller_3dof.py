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
        self.x = self.model.T_ee_w_fn(self.q[0], self.q[1], self.q[2])[0:3, 3]
        self.x_prev = np.zeros(3)

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
        self.path_pub = rospy.Publisher(
            '/ur10_kinematic_controller_3dof/trajectory', Path, queue_size=10, latch=True
        )
        self.path = Path()
        self.path.header.frame_id = "world"


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
        #not sure if this is correct
        self.x_goal = self.x + x_goal
    
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
        self.start_time = rospy.Time.now()

        # Optional: publish the trajectory for visualization
        now = rospy.Time.now()
        self.path.header.stamp = now
        self.path.poses = []
        num_samples = 50
        for i in range(num_samples + 1):
            t = (float(i) / num_samples) * self.duration
            x_des, _, _ = self.trajectory.update(t)
            pose = PoseStamped()
            pose.header.frame_id = self.path.header.frame_id
            pose.header.stamp = now + rospy.Duration.from_sec(t)
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (x_des)  
            pose.pose.orientation.w = 1.0
            self.path.poses.append(pose)
        self.path_pub.publish(self.path)


    def update(self, joint_states: np.ndarray,
               time: rospy.Time, dt: rospy.Duration) -> np.ndarray:
        """ Update the controller

        Args:
            joint_states (np.ndarray): current joint states from robot feedback [q, qdot, qddot] (3 x n)
            time (rospy.Time): current ros time
            dt (rospy.Duration): time interval

        Returns:
            np.ndarray: controller output u
        """

        # retrieve joint positions and velocities
        self.q = joint_states[0, :]
        self.qdot = joint_states[1, :]

        # keep the planned trajectory visible
        if self.path.poses:
            self.path.header.stamp = time
            self.path_pub.publish(self.path)

        ########################################################################
        # TODO: Implement the kinematic controller

        # end-effector position
        # save previous position
        self.x_prev = self.x.copy()
        T_ee_w = self.model.T_ee_w_fn(self.q[0], self.q[1], self.q[2])
        self.x = T_ee_w[0:3, 3]

        # end-effector jacobian
        J_ee_w = self.model.J_ee_w_fn(self.q[0], self.q[1], self.q[2])

        # extract jacobian for linear velocity only
        Jv_ee_w = J_ee_w[3:6, :]

        # compute the manipulability index
        JJ_T = Jv_ee_w @ Jv_ee_w.T
        w = np.sqrt(np.linalg.det(JJ_T)) if np.linalg.matrix_rank(JJ_T) == Jv_ee_w.shape[1] else 0.0

        # get desired cartesian states from the trajectory
        t_elapsed = (time - self.start_time).to_sec()
        x_des, v_des, a_des = self.trajectory.update(t_elapsed)
    
        # is this the safty mechanism asked for in the second last question
        if w < self.w_threshold:
            rospy.logwarn("Singular configuration detected! Stopping robot")
            u = np.zeros(3)
        elif np.isclose(self.x, self.x_goal, atol=1e-4).all():
            rospy.loginfo("Goal reached!")
            u = np.zeros(3)
        else:
            r_dot = v_des
            u = np.linalg.inv(Jv_ee_w) @ r_dot     

        return u
    
# Question 5.2.7: 
# The safety mechanism of the last item is radical and, in a real scenario, impractical.
# Which other mechanism can be used to avoid stopping the robot and try to reach
# the target position as close as possible? Please elaborate your answer.

# Answer:
# Instead of stopping the robot when w < w_threshold, a more practical method is
# to use the damped least-squares (DLS) inverse kinematics:
#
#     q_dot = J^T (J J^T + λ^2 I)^(-1) r_dot
#
# The damping term λ^2 prevents the matrix from becoming singular.
# This avoids large joint velocities near singularities and keeps the controller stable.
# The robot will slow down near singular configurations but still move toward the goal,
# reaching a position as close as possible instead of stopping abruptly.
