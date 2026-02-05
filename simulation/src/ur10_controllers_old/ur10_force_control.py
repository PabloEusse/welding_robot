import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import WrenchStamped
import numpy as np
from visualization_msgs.msg import MarkerArray
from ur10_model.ur10_model_3dof import UR10Model3DOF


class UR10Controller():

    class Listener():
        def __init__(self, topic):
            self._latest = None
            self._lock = False

            rospy.Subscriber(
                topic,
                MarkerArray,
                self._callback,
                queue_size=1
            )

        def _callback(self, msg):
            self._latest = msg

        def get_latest(self):
            return self._latest
        
    def __init__(self, model: UR10Model3DOF, q_init: np.ndarray = np.zeros((3, 1))) -> None:
        self.model = model
        self.q = q_init

        ########################################################################
        # ROS connection
        ########################################################################
        # TODO: Subscribe to target position and obstacle position
        self.target_pos_sub = self.Listener("/target_marker")
        self.obstacle_pos_sub = self.Listener("/obstacle_marker")
        self.target_pos = np.full((3), 10)
        self.obstacle_pos = np.full((3), 10)
        print(self.target_pos)
        # Optional: Publish repulsive forces for visualization 
        # (Use the message geometry_msgs/WrenchStamped) )
        self.ee_wrench_pub = None
        self.joint1_wrench_pub = None
        self.joint2_wrench_pub = None
        self.joint3_wrench_pub = None
        ########################################################################
        # Control Gains
        ########################################################################
        # TODO: Tune the control gains here
        self.Kp = np.diag([0, 0, 0])
        self.Kd = np.diag([0, 0, 0])

        # positions initialization
        self.target_pos = np.zeros(3)
        self.obstacle_pos = np.zeros(3)

    def update(self, joint_states: np.ndarray, cartesian_states: np.ndarray, t: rospy.Time, dt: rospy.Duration) -> np.ndarray:
        tau = self.force_controller(joint_states, cartesian_states, dt)
        return tau

    def get_marker_pos(self):
        msg = self.target_pos_sub.get_latest()
        if msg is not None:
            m = msg.markers[-1]
            pos = m.pose.position
            self.target_pos = np.array([pos.x, pos.y, pos.z]).flatten()

        msg = self.obstacle_pos_sub.get_latest()
        if msg is not None:
            m = msg.markers[-1]
            pos = m.pose.position
            self.obstacle_pos = np.array([pos.x, pos.y, pos.z]).flatten()
    def force_controller(self, joint_states: np.ndarray, cartesian_states, dt: rospy.Duration) -> np.ndarray:

        q = joint_states[0, :].reshape(-1, 1).flatten()
        qdot = joint_states[1, :].reshape(-1, 1).flatten()
        qddot = joint_states[2, :].reshape(-1, 1).flatten()
        x = cartesian_states[0, :].reshape(-1, 1).flatten()
        xdot = cartesian_states[1, :].reshape(-1, 1).flatten()
        xddot = cartesian_states[2, :].reshape(-1, 1).flatten()
        self.get_marker_pos()
        ########################################################################
        # Attractive Force  
        ########################################################################
        # TODO: Calculate the virtual attractive force based on the position error
        # to the target
        # end-effector position

        # position error
        distance = self.target_pos - x
        # end-effector Jacobian
        j_ee = self.model.J_ee_w_fn(*q)[3:, :]
        # end-effector velocity

        # virtual attractive force Virtual atractive force. 
        # Function using the error in position and a damping in velocity   
        friction = np.diag([1.0, 1.0, 1.0])
        f_constant = 10
        F_att = distance * f_constant - friction @ xdot

        attractive_torques = j_ee.T @ F_att
        ########################################################################
        # Repulsive Forces
        ########################################################################
        # TODO: Calculate the repulsive forces based on the distance to the obstacle
        # from joints 1-3 and the end-effector
        # Hint: You need to add the calculation of the transformations and Jacobians
        # for each joint in your model.

        # cartesian positions of each joint
        x_1_w = self.model.T_1_w_fn(*q)[:-1, -1]
        x_2_w = self.model.T_2_w_fn(*q)[:-1, -1]
        x_3_w = self.model.T_3_w_fn(*q)[:-1, -1]

        # joint Jacobians (linear part)
        J_1_w = self.model.J_1_w_fn(*q)[3:, :]
        J_2_w = self.model.J_2_w_fn(*q)[3:, :]
        J_3_w = self.model.J_3_w_fn(*q)[3:, :]

        # threshold distance
        d_min = 0.5
        # distances of obstacle to joints and ee w.r.t. world frame
        epsilon = 0.001
        d1_w = self.obstacle_pos - x_1_w + epsilon
        d1_norm = np.linalg.norm(d1_w)
        d2_w = self.obstacle_pos - x_2_w + epsilon
        d2_norm = np.linalg.norm(d2_w)
        d3_w = self.obstacle_pos - x_3_w + epsilon
        d3_norm = np.linalg.norm(d3_w)
        d4_w = self.obstacle_pos - x + epsilon
        d4_norm = np.linalg.norm(d4_w)
        
        rep_force_constant = 2
        print(self.obstacle_pos)
        F1_rep = np.zeros(3)
        if d1_norm < d_min:
            F1_rep = (1/d1_w) * rep_force_constant

        F2_rep = np.zeros(3)
        if d2_norm < d_min:
            F2_rep = (1/d2_w) * rep_force_constant
        
        F3_rep = np.zeros(3)
        if d3_norm < d_min:
            F3_rep = (1/d3_w) * rep_force_constant
        
        F4_rep = np.zeros(3)
        if d4_norm < d_min:
            F4_rep = (1/d4_w) * rep_force_constant
        
        repulsive_torques = J_1_w.T @ F1_rep + J_2_w.T @ F2_rep + J_3_w.T @ F3_rep + j_ee.T @ F4_rep
        # print(repulsive_torques)
        # print(dt)
        ########################################################################
        # Optional: Publish repulsive forces for visualization using WrenchStamped msgs
        # Hint: For visualization the forces on each joint / ee, you need to 
        # rotate the forces from world frame back to the body frame
        
        
        ########################################################################
        # Gravity Compensation
        ########################################################################
        # TODO: Calculate the gravity vector
        G = self.model.G_fn(*q).astype(float).flatten()


        ########################################################################
        # TODO: Sum all forces with their corresponding Jacobians, and gravity
        tau = np.array(G + attractive_torques - repulsive_torques)
        tau = tau.reshape(3, 1)
        return tau
    
    # TODO: Define callback functions for target and obstacle positions

