import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import numpy as np
from enum import Enum
from geometry_msgs.msg import Vector3
from ur10_model.ur10_model_3dof import UR10Model3DOF


class UR10Controller():
    """ Passitivity-based joint space controller for UR10 Robot (3 DOF)
    """

    class State(Enum):
        INIT = 1
        REGULATE = 2
        TRACK_SLOW = 3
        TRACK_FAST = 4
        END = 5

    class Controller(Enum):
        PD = 1
        PD_G = 2
        PID = 3

    def __init__(self, model: UR10Model3DOF, q_init: np.ndarray = np.zeros((3, 1))) -> None:
        self.model = model
        self.q = q_init

        self.state = self.State.INIT
        self.controller = self.Controller.PD_G
        self.t_start = rospy.Time.now()

        ########################################################################
        # ROS Publishers
        ########################################################################
        self.joint_states_des_pub = rospy.Publisher(
            "/joint_states_des", Float32MultiArray, queue_size=10)
        self.lyapunov_pub = rospy.Publisher(
            "/lyapunov", Float32, queue_size=10)
        self.steady_state_error_pub = rospy.Publisher(
            "/steady_state_error", Float32MultiArray, queue_size=10)

        ########################################################################
        # Control Gains
        ########################################################################
        # TODO: Tune the control gains here
        self.integral_error_acc = 0
        self.e_prev = np.array([0, 0, 0])        
        self.Kp_PD = np.diag([50, 200, 100])
        self.Kd_PD = np.diag([1, 5, 0.5])
        
        self.Kp_PDG = np.diag([50, 50, 50])
        self.Kd_PDG = np.diag([1, 1, 1])

        self.Kp_PID = np.diag([50, 70, 50])
        self.Kd_PID = np.diag([1, 5, 1])    
        self.Ki_PID = np.diag([20, 70, 20])
        self.initialized = False
        # Initialize control gains
        self.Kp = self.Kp_PD
        self.Kd = self.Kd_PD
        self.Ki = np.zeros((3, 3))
        self.initial_pose = np.array([0, 0, 0])
        self.maintain_pose = np.array([0, -np.pi / 2, -np.pi / 2])
        #print(self.Kp)
        ########################################################################
        # Time for State Transition
        ########################################################################
        # TODO: Set the time for each state
        self.state = self.State.INIT
        self.t_INIT = 2
        self.t_REGULATE = 5
        self.t_TRACK_SLOW = 15
        self.t_TRACK_FAST = 15
        self.t0 = rospy.Time.now().to_sec()
        self.last_t_regulate = None
        self.last_t_slow = None
        self.last_t_fast = None
        self.last_q = None
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

    def state_machine(self, state: Enum, t_elapsed: float):
        """
        State Machine:

        Sequence: 
          PD: REGULATE -> TRACK_SLOW -> TRACK_FAST 
          PD_G: REGULATE -> TRACK_SLOW -> TRACK_FAST 
          PID: REGULATE -> TRACK_SLOW -> TRACK_FAST 
        """
        if t_elapsed > self.t_INIT + self.t_REGULATE + self.t_TRACK_SLOW + self.t_TRACK_FAST:
            self.state = self.State.END
        elif t_elapsed > self.t_INIT + self.t_REGULATE + self.t_TRACK_SLOW:
            self.state = self.State.TRACK_FAST
        elif t_elapsed > self.t_INIT + self.t_REGULATE:
            self.state = self.State.TRACK_SLOW
        elif t_elapsed > self.t_INIT:
            self.state = self.State.REGULATE
        else:
            self.state = self.State.INIT

    def trajectory(self, state: Enum, t: rospy.Time) -> np.ndarray:
        """Generate Desired Joint States

        Args:
            state (Enum): state of the state machine
            t (rospy.Time): current ros time

        Returns:
            np.ndarray: desired joint states [q_des, qdot_des, qddot_des] (3 x n)
        """

        # TODO: Implement the desired joint states based on the state machine
        q_des = np.zeros(self.model.nq)
        qdot_des = np.zeros(self.model.nq)
        qddot_des = np.zeros(self.model.nq)
        f_slow = 0.5
        f_fast = 1.5
        A = np.array([0.5, 0.5, 0.5])
        if self.state == self.State.INIT:
            q_des = self.q + ((self.maintain_pose - self.q) / self.t_INIT) * (t - self.t0)
        elif self.state == self.State.REGULATE:
            q_des = self.maintain_pose
            self.last_t_regulate = t
        elif self.state == self.State.TRACK_SLOW:
            q_des = self.maintain_pose + np.sin(f_slow * (t - self.last_t_regulate)) * A
            self.last_q = q_des
            self.last_t_slow = t
        elif self.state == self.State.TRACK_FAST:
            q_des = self.last_q + np.sin(f_fast * (t - self.last_t_slow)) * A
            print(q_des)
            self.last_t_fast = t
            #self.last_q = q_des
        else:
            last = self.last_q + np.sin(f_fast * (self.last_t_fast - self.last_t_slow)) * A
            q_des = last + ((self.q - last) / self.t_INIT) * (t - self.last_t_fast) if t - self.last_t_fast < self.t_INIT else self.q.flatten()
        joint_states_des = np.row_stack((q_des, qdot_des, qddot_des))


        # TODO: Publish Desired Joint States
        # Remember the second joint is offset by -pi/2
        #joint_msg = JointState()
        #self.joint_states_des_pub.publish(joint_msg)

        return joint_states_des

    ########################################################################
    #                        Joint Space Controller                        #
    ########################################################################

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

        q_des = joint_states_des[0, :].reshape(-1, 1).flatten()
        qdot_des = joint_states_des[1, :].reshape(-1, 1).flatten()
        qddot_des = joint_states_des[2, :].reshape(-1, 1).flatten()

        # TODO: Implement the controllers for PD, PD_G, PID
        # hint: use if-else or switch-case to select the controller
        M = self.model.M_fn(q[0], q[1], q[2]).astype(float)
        G = self.model.G_fn(q[0], q[1], q[2]).astype(float)
        tau = np.zeros((self.model.nq, 1))
        e = q_des - q
        if not self.initialized:
            self.e_prev = e
            self.initialized = True
        e_d =  (e - self.e_prev) / dt #qdot_des - qdot
        self.e_prev = e
        self.integral_error_acc += e * dt 
        e_i = self.integral_error_acc
        self.q_prev = q
        potential = 0
        if controller_type == self.Controller.PD:
            tau = e @ self.Kp + e_d @ self.Kd
            potential = self.Kp
        elif controller_type == self.Controller.PD_G:
            gravity_torques = G.flatten()
            tau = e @ self.Kp_PDG + e_d @ self.Kd_PDG + gravity_torques
            potential = self.Kp_PDG
        elif controller_type == self.Controller.PID:
            tau = e @ self.Kp_PID + e_d @ self.Kd_PID + e_i @ self.Ki_PID
            potential = self.Kp_PID
        #print(e_i)
        tau = tau.reshape(3, 1)
        max_torque = np.array([200, 200, 200]).reshape(3, 1)
        # TODO: Calculate Lyapunov function and publish
        V = 1 / 2 * (qdot.T @ M @ qdot) + 1/2 * (e.T @ potential @ e)
        self.lyapunov_pub.publish(V)
        # TODO: Calculate steady state error and publish
        err = e
        msg = Float32MultiArray()
        msg.data = e   # list or 1D numpy array
        self.steady_state_error_pub.publish(msg)    
        
        msg = Float32MultiArray()
        msg.data = q_des + np.array([0, -np.pi / 2, 0])   # list or 1D numpy array
        self.joint_states_des_pub.publish(msg) 
        tau_limited = np.sign(tau) * np.minimum(np.abs(tau), max_torque)
        return  tau
