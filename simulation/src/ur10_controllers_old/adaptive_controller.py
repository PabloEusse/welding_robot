# Implement a joint error space controller for T7 task 1-2
from ur10_model.ur10_model_3dof import UR10Model3DOF
import numpy as np
from enum import Enum
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
np.set_printoptions(linewidth=np.inf)
class AdaptiveJointErrorSpaceController():
    class State(Enum):
        INIT = 1
        REGULATE = 2
        TRACK_SLOW = 3
        TRACK_FAST = 4
        END = 5

    class Controller(Enum):
        SIMPLE_BREAK = 1
        PD = 2
        PID = 3

    def __init__(self,  model: UR10Model3DOF, q_init: np.ndarray = np.zeros((3, 1))):
        self.model = model
        self.q_init = q_init
        self.q = q_init
        self.state = self.State.INIT
        self.controller = self.Controller.PD
        self.t_start = rospy.Time.now()
        self.theta = np.full((14), .0)
        self.ro = np.full((14), 0.05)
        ########################################################################
        # ROS Publishers
        ############################################2###########################
        self.joint_states_des_pub = rospy.Publisher(
            "/joint_states_des", Float32MultiArray, queue_size=10)
        self.lyapunov_pub = rospy.Publisher(
            "/lyapunov", Float32, queue_size=10)
        self.steady_state_error_pub = rospy.Publisher(
            "/steady_state_error", Float32MultiArray, queue_size=10)
        self.integral_error_acc = 0     
        self.Kp_PD = np.diag([30, 30, 30, 2, .1, .01])
        self.Kd_PD = np.diag([30, 30, 30, 2, .1, .01])  

        self.Kp_PID = np.diag([2, 2, 2, 2, 2, 2])
        self.Kd_PID = np.diag([3, 3, 3, 3, 3, 3]) 
        self.Ki_PID = np.diag([1.0, 1.0, 1.0, 1, 1, 1])
        self.e_acc = np.zeros(self.model.nq)
        self.state = self.State.INIT
        self.t_INIT = 120
        self.t_REGULATE = 0
        self.t_TRACK_SLOW = 50
        self.t_TRACK_FAST = 50
        self.last_t = rospy.Time.now().to_sec()
        self.last_q = q_init
        self.initial_pose = np.array([0, 0, 0, 0, 0, 0])
        self.maintain_pose = np.array([0, 0, -np.pi/2, 0, 0, 0])
        self.t0 = rospy.Time.now().to_sec()

    def state_machine(self, state: Enum, t_elapsed: float):
        """
        State Machine:

        Sequence: 
          PD: REGULATE -> TRACK_SLOW -> TRACK_FAST 
          PD_G: REGULATE -> TRACK_SLOW -> TRACK_FAST 
          PID: REGULATE -> TRACK_SLOW -> TRACK_FAST 
        """
        if t_elapsed > self.t_INIT + self.t_REGULATE + self.t_TRACK_SLOW + self.t_TRACK_FAST and self.state == self.State.TRACK_FAST:
            self.state = self.State.END
            self.last_t = rospy.Time.now().to_sec()
            self.last_q = self.q
        elif t_elapsed > self.t_INIT + self.t_REGULATE + self.t_TRACK_SLOW and self.state == self.State.TRACK_SLOW:
            self.state = self.State.TRACK_FAST
            self.last_t = rospy.Time.now().to_sec()
            self.last_q = self.q
        elif t_elapsed > self.t_INIT + self.t_REGULATE and self.state == self.State.REGULATE:
            self.state = self.State.TRACK_SLOW
            self.last_t = rospy.Time.now().to_sec()
            self.last_q = self.q
        elif t_elapsed > self.t_INIT and self.state == self.State.INIT:
            self.state = self.State.REGULATE
            self.last_t = rospy.Time.now().to_sec()
            self.last_q = self.q

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
        f_slow = 0.5
        f_fast = 1.5
        f_slow_start = 0.055
        A = np.array([.5, .5, .5, .5, .5, .5])    
        if self.state == self.State.INIT:
            q_des = self.last_q + np.sin(f_slow_start * (t - self.last_t)) * A
            qdot_des = np.cos(f_slow_start * (t - self.last_t)) * A * f_slow_start
            qddot_des = -np.sin(f_slow_start * (t - self.last_t)) * A * f_slow_start ** 2
            # q_des = self.maintain_pose
            # qdot_des = np.zeros(self.model.nq)
            # qddot_des = np.zeros(self.model.nq)
        elif self.state == self.State.REGULATE:
            q_des = self.maintain_pose
            qdot_des = np.zeros(self.model.nq)
            qddot_des = np.zeros(self.model.nq)
        elif self.state == self.State.TRACK_SLOW:
            q_des = self.last_q + np.sin(f_slow * (t - self.last_t)) * A
            qdot_des = np.cos(f_slow * (t - self.last_t)) * A * f_slow
            qddot_des = -np.sin(f_slow * (t - self.last_t)) * A * f_slow ** 2
        elif self.state == self.State.TRACK_FAST:
            q_des = self.last_q + np.sin(f_fast * (t - self.last_t)) * A
            qdot_des = np.cos(f_fast * (t - self.last_t)) * A * f_fast
            qddot_des = -np.sin(f_fast * (t - self.last_t)) * A * f_fast ** 2
        else:
            q_des = self.initial_pose
            qdot_des = np.zeros(self.model.nq)
            qddot_des = np.zeros(self.model.nq)
        joint_states_des = np.row_stack((q_des, qdot_des, qddot_des))


        # TODO: Publish Desired Joint States
        # Remember the second joint is offset by -pi/2
        #joint_msg = JointState()
        #self.joint_states_des_pub.publish(joint_msg)

        return joint_states_des
    def update(self, joint_states: np.ndarray, t: rospy.Time, dt: rospy.Duration) -> np.ndarray:
        """Update the controller

        Args:
            joint_states (np.ndarray): joint states from robot feedback [q, qdot, qddot] (3 x n)
            t (rospy.Time): current ros time
            dt (rospy.Duration): time interval

        Returns:
            np.ndarray: torque commands (n x 1)     
        """
        # TODO: update state machine
        t_elapsed = t - self.t0
        self.state_machine(None, t_elapsed)

        # TODO: update trajectory
        joint_states_des = self.trajectory(None, t)
        # TODO: update controller and return torque commands
        tau = self.joints_space_controller(joint_states, joint_states_des, self.controller, dt)

        return tau
    def joints_space_controller(self, joint_states: np.ndarray, joint_states_des: np.ndarray,
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
        self.q = q
        q_des = joint_states_des[0, :].reshape(-1, 1).flatten()
        qdot_des = joint_states_des[1, :].reshape(-1, 1).flatten()
        qddot_des = joint_states_des[2, :].reshape(-1, 1).flatten()

        # TODO: Implement the controllers for PD, PD_G, PID
        # hint: use if-else or switch-case to select the controller
        delta_q = q - q_des
        delta_qdot = qdot - qdot_des
        tau = np.zeros((self.model.nq, 1))
        if self.controller == self.Controller.PD:
            qrdot = qdot_des - self.Kp_PD @ (delta_q)
            qrddot = qddot_des - self.Kd_PD @ (delta_qdot)
            S_q = qdot - qrdot  
            Yr = self.model.Y_fn(*q)
            self.theta -= (self.ro * (Yr.T @ S_q)) * dt
            tau = -self.Kd_PD @ S_q + Yr @ self.theta#np.array(self.model.theta).astype(float).flatten()
        elif self.controller == self.Controller.PID:
            qrdot = qdot_des - self.Kp_PID @ (delta_q) - self.Ki_PID @ (self.e_acc)
            qrddot = qddot_des - self.Kp_PID @ (delta_qdot) - self.Ki_PID @ (delta_q)
            S_q = qdot - qrdot
            Yr = self.model.Y_fn(*q, *qdot, *qddot, *qrdot, *qrddot)
            self.theta -= (self.ro * (Yr.T @ S_q)) * dt
            tau = -self.Kd_PID @ S_q + Yr @ self.theta
        # print(Yr @ self.theta, "tau")
        # print(-self.Kd_PD @ S_q, "other")
        # print(delta_q)
        # print(self.theta)
        # print(self.state)
        self.e_acc += delta_q * dt
        #tau = tau.reshape(self.model.nq, 1)
        msg = Float32MultiArray()
        msg.data = delta_q   # list or 1D numpy array
        self.steady_state_error_pub.publish(msg)    
        
        msg = Float32MultiArray()
        msg.data = q_des + np.array([0, -np.pi / 2, 0, -np.pi, 0, 0])   # list or 1D numpy array
        self.joint_states_des_pub.publish(msg)
        return tau