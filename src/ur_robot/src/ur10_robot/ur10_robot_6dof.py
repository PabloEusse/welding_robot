import rospy
import numpy as np
from ur10_model.ur10_model_3dof import UR10Model3DOF
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_matrix
from geometry_msgs.msg import TransformStamped
from msbrdm_libs.utils import joint_states_integration 
from sensor_msgs.msg import JointState
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
class UR10Robot3DOF():
    """ A simulated UR10 robot that visualizes joint states and updates 
    transformations with the model
    """

    def __init__(self, model: UR10Model3DOF, q_init: np.ndarray = None) -> None:
        # initialization
        self.model = model

        self.q = np.zeros(self.model.nq)
        if q_init is not None:
            self.q = q_init
        self.qdot = np.zeros(self.model.nq)
        self.qddot = np.zeros(self.model.nq)
        #######################################################################
        # TODO: Initialize the joint states publisher
        self.joint_states_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.br = TransformBroadcaster()

        #######################################################################
        # TODO: Define the end-effector velocity publishers
        # Type: geometry_msgs/Twist
        self.ee_vel_from_J_pub = rospy.Publisher("/ee_vel_from_J", Twist, queue_size=10)
        # Type: geometry_msgs/Vector3
        self.ee_vel_from_num_diff_pub = rospy.Publisher("/ee_vel_from_diff", Vector3, queue_size=10)
        self.path_pub = rospy.Publisher("/trajectory_path", Path, queue_size=10)
        #######################################################################
        # ellispoid marker publisher
        self.marker_pub = rospy.Publisher("/manipulability_ellipsoid", Marker, queue_size=10)
        self.ellipoid_marker = Marker()
        self.ellipoid_marker.header.frame_id = "world"
        self.ellipoid_marker.header.stamp = rospy.Time.now()
        self.ellipoid_marker.type = Marker.SPHERE

        self.ellipoid_marker.color.a = 0.5
        self.ellipoid_marker.color.r = 0.0
        self.ellipoid_marker.color.g = 1.0
        self.ellipoid_marker.color.b = 0.0

        self.path_pub = rospy.Publisher("/ee_path", Path, queue_size=10)
        self.w = 10
        self.ee_path = Path()
        self.ee_path.header.frame_id = "world"   # or base_link

        #######################################################################
        # Initialize end-effector position for numerical differentiation
        self.x_ee_w = np.zeros((3, 1))
        self.x_ee_w_prev = self.x_ee_w

    def set_joint_states(self, q: np.ndarray, qdot: np.ndarray) -> None:
        """Set the joint states of the robot

        Args:
            q (np.ndarray): joint positions (n x 1)
            qdot (np.ndarray): joint velocities (n x 1)
        """
        self.q = q
        self.qdot = qdot

    def get_joint_states(self) -> np.ndarray:
        """ Get the joint states

        Returns:
            np.ndarray: joint states (3 x n)
            first row: joint positions
            second row: joint velocities
            third row: joint accelerations
        """
        joint_states = np.row_stack((self.q, self.qdot, self.qddot))
        return joint_states

    def command_vel_interface(self, u: np.ndarray, dt: rospy.Duration) -> None:
        self.q += u
        self.qdot = u / dt.to_sec()
        self.get_message()
    def get_message(self, q=None, qdot=None):
        js_msg = JointState()
        js_msg.header.stamp = rospy.Time.now()
        js_msg.name = ['ursa_shoulder_pan_joint', 'ursa_shoulder_lift_joint', 'ursa_elbow_joint', "ursa_wrist_1_joint", "ursa_wrist_2_joint", "ursa_wrist_1_joint"]
        offset = np.array([0, np.pi/2, 0, np.pi/2, 0, 0])
        js_msg.position = (self.q.flatten() - offset).tolist()
        js_msg.velocity = self.qdot.flatten().tolist()
        self.joint_states_pub.publish(js_msg)

    def broadcast_tf(self, T, frame_id ="world", child="joint1"):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = rospy.Time.now()
        tf_msg.header.frame_id = frame_id
        tf_msg.child_frame_id = child

        tf_msg.transform.translation.x = T[0, 3]
        tf_msg.transform.translation.y = T[1, 3]
        tf_msg.transform.translation.z = T[2, 3]

        q = quaternion_from_matrix(T) # note that the function expects a 4x4 matrix
        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]

        self.br.sendTransform(tf_msg)
    def update_kinematics(self,) -> None:
        """Update the forward kinematics of the robot with joint states
        """
        #######################################################################
        # TODO: Publish the joint states (self.q, self.qdot)as a JointState message
        # Note: there is a -pi/2 offset in the second joint
        #self.get_message(self.q, self.qdot)
        #######################################################################
        # TODO: Plug in the numerical joint states to the functions T(q) in the model
        # to get the numerical transformation matrices
        # T_ee_w
        self.T_ee_w = self.model.T_ee_w_fn(*self.q)
        ####################################################################### 
        # TODO: Broadcast the transformation matrices as tf frames
        # T_ee_w
        self.broadcast_tf(self.T_ee_w)
        self.broadcast_tf(self.model.T_6_w_fn(*self.q), child="cm6")
        self.broadcast_tf(self.model.T_5_w_fn(*self.q), child="cm5")
        self.broadcast_tf(self.model.T_4_w_fn(*self.q), child="cm4")
        self.broadcast_tf(self.model.T_3_w_fn(*self.q), child="cm3")
        self.broadcast_tf(self.model.T_2_w_fn(*self.q), child="cm2")
        self.broadcast_tf(self.model.T_1_w_fn(*self.q), child="cm1")
        #######################################################################

    def get_ee_w(self):
        return self.model.T_ee_w_fn(self.q[0], self.q[1], self.q[2])[:-1, -1]
    
    def get_w_index_and_jacobian(self):
        J_ee_w = self.model.J_ee_w_fn(self.q[0], self.q[1], self.q[2])[3:, :]
        return np.sqrt(np.linalg.det(J_ee_w @ J_ee_w.T)), J_ee_w
    
    def get_ee_w_dot(self):
        J_ee_w = self.model.J_ee_w_fn(self.q[0], self.q[1], self.q[2])
        
        # Calculate the end-effector Twist in the world frame V_ee_w
        V_ee_w = J_ee_w @ self.qdot
        return V_ee_w[3:]
    
    def get_ee_w_ddot(self):
        J_ee_w = self.model.J_ee_w_fn(self.q[0], self.q[1], self.q[2])
        Jd_ee_w = self.model.Jd_ee_w_fn(self.q[0], self.q[1], self.q[2], self.qdot[0], self.qdot[1], self.qdot[2])
        return (J_ee_w @ self.qddot + Jd_ee_w @ self.qdot)[3:] 
    
    def update_differential_kinematics(self, dt: float) -> None:
        """Update the differential kinematics of the robot with the given joint states

        Args:
            dt (float): time step
        """
        ########################################################################
        # TODO: Calculate the end-effector twist from Jacobian and publish as a
        # geometry_msgs/Twist message

        # Calculate end-effector Jacobian in the world frame J_ee_w
        J_ee_w = self.model.J_ee_w_fn(*self.q)
        
        # Calculate the end-effector Twist in the world frame V_ee_w
        V_ee_w = J_ee_w @ self.qdot

        # Publish the end-effector Twist as a geometry_msgs/Twist message
        twist_msg = Twist()
        twist_msg.angular.x = V_ee_w[0]
        twist_msg.angular.y = V_ee_w[1]
        twist_msg.angular.z = V_ee_w[2]
        twist_msg.linear.x = V_ee_w[3]
        twist_msg.linear.y = V_ee_w[4]
        twist_msg.linear.z = V_ee_w[5]
        self.ee_vel_from_J_pub.publish(twist_msg)
        
        ########################################################################
        # TODO: Calculate the end-effector linear velocity from numerical
        # differentiation of the position and publish as a geometry_msgs/Vector3
        # message

        # End-effector position
        self.x_ee_w = self.T_ee_w[:3, 3]

        # Numerical differentiation
        v_ee_w_num = (self.x_ee_w - self.x_ee_w_prev) / dt

        # Save the current end-effector position for the next iteration
        self.x_ee_w_prev = self.x_ee_w

        # Publish the end-effector linear velocity as a geometry_msgs/Vector3 message
        vector_message = Vector3()
        vector_message.x = v_ee_w_num[0]
        vector_message.y = v_ee_w_num[1]
        vector_message.z = v_ee_w_num[2]
        self.ee_vel_from_num_diff_pub.publish(vector_message)   

        ########################################################################
        # Manipulability analysis
        # TODO: plug in the linear part of the Jacobian for manipulability analysis
        self.manipulatbility_analysis(J_ee_w)

    def publish_ee_path(self, T_ee):
        """
        Publishes EE path from a 4x4 homogeneous transformation matrix.

        Args:
            T_ee (np.ndarray): 4x4 homogeneous transformation of the EE
        """

        assert T_ee.shape == (4, 4), "T_ee must be 4x4"

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.ee_path.header.frame_id

        # Translation
        pose.pose.position.x = T_ee[0, 3]
        pose.pose.position.y = T_ee[1, 3]
        pose.pose.position.z = T_ee[2, 3]

        # Rotation (matrix → quaternion)
        q = quaternion_from_matrix(T_ee)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        # Append to path
        self.ee_path.header.stamp = pose.header.stamp
        self.ee_path.poses.append(pose)

        self.path_pub.publish(self.ee_path)
    def set_w(self):
        J_ee_w = self.model.J_ee_w_fn(self.q[0], self.q[1], self.q[2])
        J_v = J_ee_w[3:, :]
        self.w = np.sqrt(np.linalg.det(J_v @ J_v.T))
        
    def manipulatbility_analysis(self, J: np.ndarray):
        """Calculate the manipulatbility index and visulize the manipulatbility ellipsoid

        Args:
            J (np.ndarray): Jacobian matrix
        """
        J_v = J[3:, :]
        ########################################################################
        # TODO: Calculate the manipulability index with the Jacobian matrix
        self.w = np.sqrt(np.linalg.det(J_v @ J_v.T))

        ########################################################################
        # TODO: Visualize the manipulatbility ellipsoid

        # Compulte the pricipal axes and axis lengths of the ellipsoid
        U, S, _ = np.linalg.svd(J_v)
        S = np.maximum(S, 1e-3)
        S = S / np.max(S)
        # Define the position of the ellipsoid
        self.ellipoid_marker.pose.position.x = self.T_ee_w[0, 3]
        self.ellipoid_marker.pose.position.y = self.T_ee_w[1, 3]
        self.ellipoid_marker.pose.position.z = self.T_ee_w[2, 3]

        
        # Define the orientation of the ellipsoid from the principal axes
        T = np.eye(4)
        T[:3, :3] = U
        q = quaternion_from_matrix(T)
        q = q / np.linalg.norm(q)

        self.ellipoid_marker.pose.orientation.x = q[0]
        self.ellipoid_marker.pose.orientation.y = q[1]
        self.ellipoid_marker.pose.orientation.z = q[2]
        self.ellipoid_marker.pose.orientation.w = q[3]

        # Define the scale of the ellipsoid from the axis lengths
        self.ellipoid_marker.scale.x = S[0]
        self.ellipoid_marker.scale.y = S[1]
        self.ellipoid_marker.scale.z = S[2]
    
        # print(U)

        # Publish the ellipsoid marker
        #self.marker_pub.publish(self.ellipoid_marker)
        return self.w

    def update_dynamics(self, tau, dt):
        self.T_cm3_w = self.model.T_cm3_w_fn(*self.q)
        self.T_cm2_w = self.model.T_cm2_w_fn(*self.q)
        self.T_cm1_w = self.model.T_cm1_w_fn(*self.q)
        self.T_ee_w = self.model.T_ee_w_fn(*self.q)
        self.T_3_w = self.model.T_3_w_fn(*self.q)
        self.T_2_w = self.model.T_2_w_fn(*self.q)
        self.T_1_w = self.model.T_1_w_fn(*self.q)
        # self.broadcast_tf(self.T_cm3_w, child="cm3")
        # self.broadcast_tf(self.T_cm2_w, child="cm2")
        # self.broadcast_tf(self.T_cm1_w, child="cm1")
        self.broadcast_tf(self.T_1_w, child="1")
        self.broadcast_tf(self.T_2_w, child="2")
        self.broadcast_tf(self.T_3_w, child="3")
        self.broadcast_tf(self.T_ee_w, child="ee")
        self.publish_ee_path(self.T_ee_w)
        M = self.model.M_fn(*self.q).astype(float)
        C = self.model.C_fn(*self.q, *self.qdot).astype(float)
        G = self.model.G_fn(*self.q).astype(float)

        # use the damping term in model (model.D) to calculate the friction 
        friction = self.model.D
        # calculate qddot
        qddot = np.linalg.pinv(M) @ (tau - C @ self.qdot.reshape((6, )) - G.reshape(6, ) - friction @ self.qdot.reshape((6, )))
        
        # integrate the joint states
        joint_states = joint_states_integration(qddot.reshape(6,), self.get_joint_states(), dt) 
        self.q = joint_states[0, :]
        self.qdot = joint_states[1, :]
        self.qddot = qddot.reshape(6)

    def publish_joint_states(self):
        self.get_message()