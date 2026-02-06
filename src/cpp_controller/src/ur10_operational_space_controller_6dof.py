
# TODO: implement a controller for T7 task 3-from ur10_model.ur10_model_3dof import UR10Model3DOF
import numpy as np
from enum import Enum
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from ur10_model.ur10_model_3dof import UR10Model3DOF
from std_msgs.msg import Float32MultiArray
from ur10_robot.ur10_robot_3dof import UR10Robot3DOF
from ur10_controllers.ur10_joint_error_space_controller_6dof import JointErrorSpaceController
from ur10_controllers.adaptive_controller import AdaptiveJointErrorSpaceController
from msbrdm_libs.spline import LinearSplineTrajectory
class SpaceController():

    class State(Enum):
        INIT = 1
        LINEAR = 2
        TRACK_SLOW = 3
        TRACK_FAST = 4
        END = 5

    class Controller(Enum):
        SIMPLE_BREAK = 1
        PD = 2
        PID = 3
    
    class Space(Enum):
        JOINT = 1
        CARTESIAN = 2

    def __init__(self,  model: UR10Model3DOF, q_init: np.ndarray = np.zeros((6, 1))):
        self.model = model
        self.robot = UR10Robot3DOF(model, q_init)
        self.X = self.robot.get_ee_w()
        self.q_init = q_init
        self.q = q_init
        self.state = self.State.INIT
        self.controller = self.Controller.PD
        self.t_start = rospy.Time.now()
        self.joint_controller = AdaptiveJointErrorSpaceController(model, q_init)#JointErrorSpaceController(model, q_init)
        ########################################################################
        # ROS Publishers
        ########################################################################
        self.joint_states_des_pub = rospy.Publisher(
            "/joint_states_des", Float32MultiArray, queue_size=10)
        self.lyapunov_pub = rospy.Publisher(
            "/lyapunov", Float32, queue_size=10)
        self.steady_state_error_pub = rospy.Publisher(
            "/steady_state_error", Float32MultiArray, queue_size=10)
        self.cartesian_pub = rospy.Publisher(
            "/cart_coor", Float32MultiArray, queue_size=10)
        self.theta = np.full((14), .0)
        self.ro = np.full((14), 0.1)
        self.integral_error_acc = 0     
        self.Kp_PD = np.diag([20, 20, 20])
        self.Kd_PD = np.diag([20, 20, 20])
        self.Kreg_PD = np.diag([20, 20, 20, 1, 1, 1])

        self.Kp_PID = np.diag([5, 5, 5])
        self.Kd_PID = np.diag([5, 5, 5])    
        self.Ki_PID = np.diag([5, 5, 5])

        self.state = self.State.INIT
        self.t_INIT = 10
        self.t_LINEAR = 10
        self.t_TRACK_SLOW = 40
        self.t_TRACK_FAST = 40
        self.last_t = rospy.Time.now().to_sec()
        self.last_q = q_init
        self.last_X = self.X
        self.start_track_pose = np.array([0, -np.pi / 3, -np.pi + np.pi / 2, 0, 0, 0])
        self.non_singular_pose = np.array([0, 0, -np.pi / 2, 0, 0, 0])
        self.t0 = rospy.Time.now().to_sec()
        self.space = self.Space.JOINT
        self.linear_trajectory = None
    def state_machine(self, state: Enum, t_elapsed: float):
        """
        State Machine:

        Sequence: 
          PD: REGULATE -> TRACK_SLOW -> TRACK_FAST 
          PD_G: REGULATE -> TRACK_SLOW -> TRACK_FAST 
          PID: REGULATE -> TRACK_SLOW -> TRACK_FAST 
        """
        if t_elapsed > self.t_INIT + self.t_LINEAR + self.t_TRACK_SLOW + self.t_TRACK_FAST and self.state == self.State.TRACK_FAST:
            self.state = self.State.END
            self.last_t = rospy.Time.now().to_sec()
            self.last_q = self.q
            self.last_X = self.X
            self.space = self.Space.JOINT
        elif t_elapsed > self.t_INIT + self.t_LINEAR + self.t_TRACK_SLOW and self.state == self.State.TRACK_SLOW:
            self.state = self.State.TRACK_FAST
            self.last_t = rospy.Time.now().to_sec()
            self.last_q = self.q
            self.last_X = self.X
            self.space = self.Space.CARTESIAN
        elif t_elapsed > self.t_INIT + self.t_LINEAR and self.state == self.State.LINEAR:
            self.state = self.State.TRACK_SLOW
            self.last_t = rospy.Time.now().to_sec()
            self.last_q = self.q
            self.last_X = self.X
            self.space = self.Space.CARTESIAN
        elif t_elapsed > self.t_INIT and self.state == self.State.INIT:
            self.state = self.State.LINEAR
            self.last_t = rospy.Time.now().to_sec()
            self.last_q = self.q
            self.last_X = self.X
            self.space = self.Space.CARTESIAN
    def closest_equivalent_angles(self, q_ref_wrapped, q_now_continuous):
        """
        Return joint targets equivalent to q_ref_wrapped
        but closest to current continuous angles q_now_continuous.
        """
        return q_ref_wrapped + 2*np.pi * np.round(
            (q_now_continuous - q_ref_wrapped) / (2*np.pi)
        )

    def trajectory(self, state: Enum, t: rospy.Time) -> np.ndarray:
        """Generate Desired Joint States

        Args:
            state (Enum): state of the state machine
            t (rospy.Time): current ros time

        Returns:
            np.ndarray: desired joint states [q_des, qdot_des, qddot_des] (3 x n)
        """

        q_des = np.zeros(self.model.nq)
        qdot_des = np.zeros(self.model.nq)
        qddot_des = np.zeros(self.model.nq)
        x_des = np.zeros(3)
        xdot_des = np.zeros(3)
        xddot_des = np.zeros(3)
        track_pos_cartesian = self.robot.model.T_ee_w_fn(*self.start_track_pose)[:-1, -1]
        omega = 2 * np.pi / self.t_TRACK_SLOW
        omega_z = omega * 15
        omega_fast = 2 * np.pi / self.t_TRACK_FAST
        omega_z_fast = omega_fast * 15
        amplitude = np.array([track_pos_cartesian[0], track_pos_cartesian[0], 0.05])   
        A = np.array([0.5, 0.5, 0.5])
        if self.state == self.State.INIT:
            #q_des = self.non_singular_pose
            q_des = ((self.non_singular_pose) / self.t_INIT) * (t - self.last_t)
        elif self.state == self.State.LINEAR:
            #track_pos_cartesian = self.robot.model.T_ee_w_fn(*self.start_track_pose)[:-1, -1]
            x_des = self.last_X + ((track_pos_cartesian - self.last_X) / self.t_LINEAR) * (t - self.last_t) 
        elif self.state == self.State.TRACK_SLOW:
            x_des = np.array([np.cos(omega * (t - self.last_t)), np.sin(omega * (t - self.last_t)), np.cos(omega_z * (t - self.last_t))]) * amplitude
            offset = t - self.last_t 
            xdot_des = amplitude * np.array([
                -np.sin(omega   * offset) * omega,
                np.cos(omega   * offset) * omega,
                -np.sin(omega_z * offset) * omega_z,
            ])
            xddot_des = np.zeros(3)
        elif self.state == self.State.TRACK_FAST:
            x_des = np.array([np.cos(omega_fast * (t - self.last_t)), np.sin(omega_fast* (t - self.last_t)), np.cos(omega_z_fast * (t - self.last_t))]) * amplitude
            offset = t - self.last_t 
            xdot_des = amplitude * np.array([
                -np.sin(omega_fast * offset) * omega_fast,
                np.cos(omega_fast * offset) * omega_fast,
                -np.sin(omega_z_fast * offset) * omega_z_fast,
            ])
            xddot_des = np.zeros(3)
        else:
            q_des = self.closest_equivalent_angles(self.non_singular_pose, self.q)
            qdot_des = np.zeros(6)
            qddot_des = np.zeros(6)

        cartesian_states_des = np.row_stack((x_des, xdot_des, xddot_des))
        joint_states_des = np.row_stack((q_des, qdot_des, qddot_des))


        # TODO: Publish Desired Joint States
        # Remember the second joint is offset by -pi/2
        #joint_msg = JointState()
        #self.joint_states_des_pub.publish(joint_msg)

        return joint_states_des, cartesian_states_des
    def update(self, cartesian_states:np.ndarray, joint_states: np.ndarray, t: rospy.Time, dt: rospy.Duration) -> np.ndarray:
        """Update the controller

        Args:
            joint_states (np.ndarray): joint states from robot feedback [q, qdot, qddot] (3 x n)
            t (rospy.Time): current ros time
            dt (rospy.Duration): time interval
        self.last_X = self.X
        self.start_track_pose = np.array([0, -np.pi / 3, -np.pi + np.pi / 2, 0, 0,

        Returns:
            np.ndarray: torque commands (n x 1)     
        """
        # TODO: update state machine
        t_elapsed = t - self.t0
        self.state_machine(None, t_elapsed)
        self.q = joint_states[0, :].reshape(-1, 1).flatten()
        self.X = cartesian_states[0, :].reshape(-1, 1).flatten()
        # TODO: update trajectory
        self.robot.q = self.q
        self.robot.set_w()
        if self.robot.w < 0.1 and self.space == self.Space.CARTESIAN:
            self.space = self.Space.JOINT
            self.state = self.State.END
        joint_states_des, cartesian_states_des = self.trajectory(None, t)
        # TODO: update controller and return torque commands
        tau = self.joint_controller.joints_space_controller(joint_states, joint_states_des, self.controller, dt) if self.space == self.Space.JOINT else self.cartesian_space_controller(cartesian_states,  joint_states, cartesian_states_des, None, dt)

        return tau
    def cartesian_space_controller(self, cartesian_states: np.ndarray,  joint_states: np.ndarray, cartesian_states_des: np.ndarray,
                            controller_type: Enum, dt: rospy.Duration) -> np.ndarray:
        """Joint Space Controller

        Args:
            joint_states (np.ndarray): current joint states from robot feedback [q, qdot, qddot] (3 x n)
            joint_states_des (np.ndarray): desired joint states [q_des, qdot_des, qddot_des] (3 x n)
            controller_type (Enum): PD, PD_G, PID
            dt (rospy.Duration): time interval

        Returns:
            np.ndarray: torque commands (n x 1)
        """

        # Rotate the joint states to column vectors for easier calculations
        q = joint_states[0, :].reshape(-1, 1).flatten()
        qdot = joint_states[1, :].reshape(-1, 1).flatten()
        qddot = joint_states[2, :].reshape(-1, 1).flatten()
        x = cartesian_states[0, :].reshape(-1, 1).flatten()
        xdot = cartesian_states[1, :].reshape(-1, 1).flatten()
        xddot = cartesian_states[2, :].reshape(-1, 1).flatten()
        x_des = cartesian_states_des[0, :].reshape(-1, 1).flatten()
        xdot_des = cartesian_states_des[1, :].reshape(-1, 1).flatten()
        xddot_des = cartesian_states_des[2, :].reshape(-1, 1).flatten()

        # TODO: Implement the controllers for PD, PD_G, PID
        # hint: use if-else or switch-case to select the controller
        delta_x = x - x_des
        self.integral_error_acc += delta_x * dt
        delta_xdot = xdot - xdot_des
        tau = np.zeros((self.model.nq, 1))
        J_ee = self.model.J_ee_w_fn(*self.q)[3:, :]
        #Jd_ee = self.model.Jd_ee_w_fn(*self.q, *qdot)
        if self.controller == self.Controller.PD:
            xrdot = xdot_des - self.Kp_PD @ delta_x
            #xrddot = xddot_des - self.Kd_PD @ (delta_xdot)
            qrdot = np.linalg.pinv(J_ee) @ xrdot
            # Clip joint velocity to safe limits
            max_joint_vel = 0.5  # rad/s
            qrdot = np.clip(qrdot, -max_joint_vel, max_joint_vel)
            #qrddot = np.linalg.pinv(J_ee) @ (xrddot - Jd_ee @ qrdot)
            S_q = np.asarray(qdot - qrdot,  dtype=np.float64)
            Yr = self.model.Y_fn(*q).astype(float)
            self.theta -= (self.ro * (Yr.T @ S_q)) * dt
            tau = -self.Kreg_PD @ S_q + Yr @ self.theta
        elif self.controller == self.Controller.PID:
            xrdot = xdot_des - self.Kp_PID @ delta_x - self.Ki_PID @ self.integral_error_acc
            xrddot = xddot_des - self.Kp_PID @ (delta_xdot) - self.Ki_PID @ delta_x
            qrdot = np.linalg.pinv(J_ee) @ xrdot
            # Clip joint velocity to safe limits
            max_joint_vel = 0.5  # rad/s
            qrdot = np.clip(qrdot, -max_joint_vel, max_joint_vel)
            qrddot = np.linalg.pinv(J_ee) @ (xrddot - Jd_ee @ qrdot)
            S_q = qdot - qrdot  
            Yr_theta = M @ qrddot + C @ qrdot + G.flatten()
            tau = -self.Kd_PD @ S_q + Yr_theta
        msg = Float32MultiArray()
        msg.data = delta_x   # list or 1D numpy array
        self.steady_state_error_pub.publish(msg)    
        
        msg = Float32MultiArray()
        msg.data = x_des   # list or 1D numpy array
        self.joint_states_des_pub.publish(msg)

        msg = Float32MultiArray()
        msg.data = x   # list or 1D numpy array
        self.cartesian_pub.publish(msg)
        return tau