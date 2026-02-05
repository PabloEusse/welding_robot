import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import WrenchStamped
import numpy as np

from ur10_model.ur10_model_3dof import UR10Model3DOF


class UR10Controller():

    def __init__(self, model: UR10Model3DOF, q_init: np.ndarray = np.zeros((3, 1))) -> None:
        self.model = model
        self.q = q_init

        # ROS connection
        self.target_pos_sub = rospy.Subscriber('/target_position', Vector3, self.target_position_callback)
        self.obstacle_pos_sub = rospy.Subscriber('/obstacle_position', Vector3, self.obstacle_position_callback)

        # Optional: Publish repulsive forces for visualization.
        self.ee_wrench_pub = rospy.Publisher('/ee_wrench', WrenchStamped, queue_size=10)
        self.joint1_wrench_pub = rospy.Publisher('/joint1_wrench', WrenchStamped, queue_size=10)
        self.joint2_wrench_pub = rospy.Publisher('/joint2_wrench', WrenchStamped, queue_size=10)
        self.joint3_wrench_pub = rospy.Publisher('/joint3_wrench', WrenchStamped, queue_size=10)

        # Control gains
        self.Kp = np.diag([64, 64, 64])
        self.Kd = np.diag([16, 16, 16])

        # Positions initialization
        self.target_pos = np.zeros((3, 1))
        self.obstacle_pos = np.zeros((3, 1))

    def update(self, joint_states: np.ndarray, t: rospy.Time, dt: rospy.Duration) -> np.ndarray:
        tau = self.force_controller(joint_states, dt)
        return tau

    def force_controller(self, joint_states: np.ndarray, dt: rospy.Duration) -> np.ndarray:
        q = joint_states[0, :].reshape(-1, 1)
        qdot = joint_states[1, :].reshape(-1, 1)

        # Attractive force
        x_ee_w = self.model.T_ee_w_fn(*q.flatten())[:3, 3].reshape(-1, 1)
        e_pos = self.target_pos - x_ee_w
        # end-effector Jacobian (linear part only; 6xN Jacobian uses [w; v])
        J_ee_w = self.model.J_ee_w_fn(*q.flatten())[3:6, :]
        xdot_ee_w = J_ee_w @ qdot
        F_att = self.Kp @ e_pos - self.Kd @ xdot_ee_w
        tau_att = J_ee_w.T @ F_att

        # Repulsive forces
        x_1_w = self.model.T_1_w_fn(*q.flatten())[:3, 3].reshape(-1, 1)
        x_2_w = self.model.T_2_w_fn(*q.flatten())[:3, 3].reshape(-1, 1)
        x_3_w = self.model.T_3_w_fn(*q.flatten())[:3, 3].reshape(-1, 1)

        J_1_w = self.model.J_1_w_fn(*q.flatten())[3:6, :]
        J_2_w = self.model.J_2_w_fn(*q.flatten())[3:6, :]
        J_3_w = self.model.J_3_w_fn(*q.flatten())[3:6, :]

        rho0 = 0.5  # influence distance
        eta = 2.0   # repulsive gain

        d1_w = self.obstacle_pos - x_1_w
        d1_norm = np.linalg.norm(d1_w)
        d2_w = self.obstacle_pos - x_2_w
        d2_norm = np.linalg.norm(d2_w)
        d3_w = self.obstacle_pos - x_3_w
        d3_norm = np.linalg.norm(d3_w)
        dee_w = self.obstacle_pos - x_ee_w
        dee_norm = np.linalg.norm(dee_w)

        def repulsive_force_point(x_i, x_obs, rho0, eta):
            r = x_i - x_obs                  # points away from obstacle
            rho = np.linalg.norm(r)
            rho = max(rho, 1e-3)
            if rho <= 1e-9 or rho > rho0:
                return np.zeros((3, 1))

            r_hat = r / rho
            F = eta * (1.0/rho - 1.0/rho0) * (1.0/(rho**2)) * r_hat
            return F

        F1_rep = np.zeros((3, 1))
        if d1_norm < rho0:
            F1_rep = repulsive_force_point(x_1_w, self.obstacle_pos, rho0, eta)
        F2_rep = np.zeros((3, 1))
        if d2_norm < rho0:
            F2_rep = repulsive_force_point(x_2_w, self.obstacle_pos, rho0, eta)
        F3_rep = np.zeros((3, 1))
        if d3_norm < rho0:
            F3_rep = repulsive_force_point(x_3_w, self.obstacle_pos, rho0, eta)
        Fee_rep = np.zeros((3, 1))
        if dee_norm < rho0:
            Fee_rep = repulsive_force_point(x_ee_w, self.obstacle_pos, rho0, eta)

        tau_rep = (J_1_w.T @ F1_rep) + (J_2_w.T @ F2_rep) + (J_3_w.T @ F3_rep) + (J_ee_w.T @ Fee_rep)

        # Optional: publish repulsive forces for visualization (world frame).
        T_0_w = self.model.T_0_w_fn()
        R_0_w = T_0_w[:3, :3]
        
        F1_rep_0 = R_0_w.T @ F1_rep
        wrench_1 = WrenchStamped()
        wrench_1.header.stamp = rospy.Time.now()
        wrench_1.header.frame_id = "world"
        wrench_1.wrench.force.x = float(F1_rep_0[0, 0])
        wrench_1.wrench.force.y = float(F1_rep_0[1, 0])
        wrench_1.wrench.force.z = float(F1_rep_0[2, 0])
        self.joint1_wrench_pub.publish(wrench_1)

        F2_rep_0 = R_0_w.T @ F2_rep
        wrench_2 = WrenchStamped()
        wrench_2.header.stamp = rospy.Time.now()
        wrench_2.header.frame_id = "world"
        wrench_2.wrench.force.x = float(F2_rep_0[0, 0])
        wrench_2.wrench.force.y = float(F2_rep_0[1, 0])
        wrench_2.wrench.force.z = float(F2_rep_0[2, 0])
        self.joint2_wrench_pub.publish(wrench_2)

        F3_rep_0 = R_0_w.T @ F3_rep
        wrench_3 = WrenchStamped()
        wrench_3.header.stamp = rospy.Time.now()
        wrench_3.header.frame_id = "world"
        wrench_3.wrench.force.x = float(F3_rep_0[0, 0])
        wrench_3.wrench.force.y = float(F3_rep_0[1, 0])
        wrench_3.wrench.force.z = float(F3_rep_0[2, 0])
        self.joint3_wrench_pub.publish(wrench_3)

        Fee_rep_0 = R_0_w.T @ Fee_rep
        wrench_ee = WrenchStamped()
        wrench_ee.header.stamp = rospy.Time.now()
        wrench_ee.header.frame_id = "world"
        wrench_ee.wrench.force.x = float(Fee_rep_0[0, 0])
        wrench_ee.wrench.force.y = float(Fee_rep_0[1, 0])
        wrench_ee.wrench.force.z = float(Fee_rep_0[2, 0])
        self.ee_wrench_pub.publish(wrench_ee)

        # Gravity compensation
        G = self.model.G_fn(*q.flatten()).astype(float)

        tau = G + tau_rep + tau_att
        return tau.reshape(-1, 1)

    def target_position_callback(self, msg: Vector3):
        self.target_pos = np.array([[msg.x], [msg.y], [msg.z]])
        print("target position updated to:", self.target_pos)

    def obstacle_position_callback(self, msg: Vector3):
        self.obstacle_pos = np.array([[msg.x], [msg.y], [msg.z]])
