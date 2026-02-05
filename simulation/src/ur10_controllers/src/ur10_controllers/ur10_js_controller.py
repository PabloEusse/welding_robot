import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import numpy as np
from enum import Enum

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
        self.controller = self.Controller.PD
        #self.controller = self.Controller.PID
        print("Initial controller type:", self.controller)

        self.t_start = rospy.Time.now()
        self.t_start_traj = rospy.Time.now()
        self.prev_state = None

        ########################################################################
        # ROS Publishers
        ########################################################################
        self.joint_states_des_pub = rospy.Publisher(
            "/joint_states_des", JointState, queue_size=10)
        self.lyapunov_pub = rospy.Publisher(
            "/lyapunov", Float32, queue_size=10)
        self.steady_state_error_pub = rospy.Publisher(
            "/steady_state_error", Float32MultiArray, queue_size=10)

        # ########################################
        # Control Gains
        ########################################################################
        # TODO: Tune the control gains here
        self.Kp_PD = np.diag([2, 2, 2])
        self.Kd_PD = np.diag([2, 2, 2])

        self.Kp_PDG = np.diag([2, 2, 2])
        self.Kd_PDG = np.diag([2, 2, 2])

        self.Kp_PID = np.diag([2, 2, 2])
        self.Kd_PID = np.diag([2, 2, 2])
        # Ki_PID need to be smaller for tracking to work^
        self.Ki_PID = np.diag([0.1, 0.1, 0.1])

        # Initialize control gains
        self.Kp = self.Kp_PD
        self.Kd = self.Kd_PD
        self.Ki = np.zeros((3, 3))

        ########################################################################
        # Time for State Transition
        ########################################################################
        # TODO: Set the time for each state
        self.t_REGULATE = 60
        self.t_TRACK_SLOW = 60
        self.t_TRACK_FAST = 60

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
        # t_elapsed = verstrichene zeit seit start des controllers
        t_elapsed = (t - self.t_start).to_sec()
        self.state_machine(self.state, t_elapsed)

        # TODO: update trajectory
        joint_states_des = self.trajectory(self.state, t)

        # TODO: update controller and return torque commands
        tau = self.joints_space_controller(joint_states, joint_states_des,
                                          self.controller, dt.to_sec())

        return tau

    def state_machine(self, state: Enum, t_elapsed: float):
        """
        State Machine:

        Sequence: 
          PD: REGULATE -> TRACK_SLOW -> TRACK_FAST 
          PD_G: REGULATE -> TRACK_SLOW -> TRACK_FAST 
          PID: REGULATE -> TRACK_SLOW -> TRACK_FAST 
        """
        # TODO: update state based on elapsed time, assign controller type and
        # control gains for each state
        # Either return new state or update self.state

        self.state = state
        # Assign state based on elapsed time
        if t_elapsed < self.t_REGULATE:
            print("currently in state REGULATE")
            self.state = self.State.REGULATE
        elif  (t_elapsed >= self.t_REGULATE) and (t_elapsed < self.t_REGULATE + self.t_TRACK_SLOW):
            print("currently in state TRACK_SLOW")
            if self.prev_state != self.State.TRACK_SLOW:
                self.t_start_traj = rospy.Time.now()
                self.prev_state = self.State.TRACK_SLOW
            self.state = self.State.TRACK_SLOW
        elif (t_elapsed >= self.t_REGULATE + self.t_TRACK_SLOW) and (t_elapsed < self.t_REGULATE + self.t_TRACK_SLOW + self.t_TRACK_FAST):
            print("currently in state TRACK_FAST")
            if self.prev_state != self.State.TRACK_FAST:
                self.t_start_traj = rospy.Time.now()
                self.prev_state = self.State.TRACK_FAST
            self.state = self.State.TRACK_FAST

        else:
            print("currently in state END")           
            self.state = self.State.END
        
        # Assign controller type and control gains for each state
        if self.controller == self.Controller.PD:
            self.Kp = self.Kp_PD
            self.Kd = self.Kd_PD
        elif self.controller == self.Controller.PD_G:
            self.Kp = self.Kp_PDG
            self.Kd = self.Kd_PDG
        elif self.controller == self.Controller.PID:
            self.Kp = self.Kp_PID
            self.Kd = self.Kd_PID
            self.Ki = self.Ki_PID

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

        if state == self.State.INIT:
            # Initial Position
            q_des = np.array([[np.pi/4], [np.pi/4], [np.pi/4]]).flatten()
            qdot_des = np.array([[0], [0], [0]]).flatten()
            qddot_des = np.array([[0], [0], [0]]).flatten()

        elif state == self.State.REGULATE:
            # Regulate to some position:
            # position that requires gravity compensation as its "upwards"
            self.q_reg = np.array([[np.pi/6], [np.pi/6], [np.pi/6]]).flatten()   
            q_des = self.q_reg
            qdot_des = np.array([[0], [0], [0]]).flatten()
            qddot_des = np.array([[0], [0], [0]]).flatten()

        elif state == self.State.TRACK_SLOW:
            # Slow Trajectory Tracking
            # Tracking function q = A* sin(omega*t) and qdot = A*omega*cos(omega*t)
            w = 0.5  # frequency parameter            
            A= np.pi/2.0
            t_elapsed = (t - self.t_start_traj).to_sec()
            q_des = self.q_reg + np.array([[A * np.sin(w * t_elapsed)],
                              [A * np.sin(w * t_elapsed)],
                              [A * np.sin(w  * t_elapsed)]]).flatten()
            qdot_des = np.array([[A * w * np.cos(w * t_elapsed)],
                                 [A * w * np.cos(w * t_elapsed)],
                                 [A * w * np.cos(w * t_elapsed)]]).flatten()
            qddot_des = np.array([[-A * w**2 * np.sin(w * t_elapsed)],
                                  [-A * w**2 * np.sin(w * t_elapsed)],
                                  [-A * w**2 * np.sin(w * t_elapsed)]]).flatten()

        elif state == self.State.TRACK_FAST:
            # Fast Trajectory Tracking
            # Tracking function q = A* sin(omega*t) and qdot = A*omega*cos(omega*t)
            w = 2.0  # frequency parameter
             A = np.pi/2.0
            t_elapsed = (t - self.t_start_traj).to_sec()
            q_des =  np.array([[A * np.sin(w * t_elapsed)],
                              [A * np.sin(w * t_elapsed)],
                              [A * np.sin(w  * t_elapsed)]]).flatten()
            qdot_des = np.array([[A  * w * np.cos(w * t_elapsed)],
                                 [A  * w * np.cos(w * t_elapsed)],
                                 [A  * w * np.cos(w * t_elapsed)]]).flatten()
            qddot_des = np.array([[-A  * w**2 * np.sin(w * t_elapsed)],
                                  [-A  * w**2 * np.sin(w * t_elapsed)],
                                  [-A  * w**2 * np.sin(w * t_elapsed)]]).flatten()
        elif state == self.State.END:
            # End Position
            q_des = np.array([[np.pi/4], [np.pi/4], [np.pi/4]]).flatten()
            qdot_des = np.array([[0], [0], [0]]).flatten()
            qddot_des = np.array([[0], [0], [0]]).flatten()

        joint_states_des = np.row_stack((q_des, qdot_des, qddot_des))

        # TODO: Publish Desired Joint States
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        
        joint_state_msg.name = [
            "ursa_shoulder_pan_joint",
            "ursa_shoulder_lift_joint",
            "ursa_elbow_joint",
            "ursa_wrist_1_joint",
            "ursa_wrist_2_joint",
            "ursa_wrist_3_joint"
        ]            
        q_full = [
            float(q_des[0]),               # shoulder_pan
            float(q_des[1] - np.pi/2),     # shoulder_lift (UR10 offset)
            float(q_des[2]),               # elbow
            -np.pi/2,                       # wrist_1 (correcting position via pi/2)
            0.0,                            # wrist_2
            0.0                             # wrist_3
        ]       
        qdot_full = [
            float(qdot_des[0]),
            float(qdot_des[1]),
            float(qdot_des[2]),
            0.0,
            0.0,
            0.0
        ]
        qddot_full = [
            float(qddot_des[0]),
            float(qddot_des[1]),
            float(qddot_des[2]),
            0.0,
            0.0,
            0.0
        ]
        joint_state_msg.position = q_full
        joint_state_msg.velocity = qdot_full
        joint_state_msg.effort = qddot_full
        self.joint_states_des_pub.publish(joint_state_msg)

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
        q = joint_states[0, :].reshape(-1, 1)
        qdot = joint_states[1, :].reshape(-1, 1)
        qddot = joint_states[2, :].reshape(-1, 1)

        q_des = joint_states_des[0, :].reshape(-1, 1)
        qdot_des = joint_states_des[1, :].reshape(-1, 1)
        qddot_des = joint_states_des[2, :].reshape(-1, 1)

        # TODO: Implement the controllers for PD, PD_G, PID
        # hint: use if-else or switch-case to select the controller
        eq = q - q_des
        edq = qdot - qdot_des
     
        if not hasattr(self, 'integral_error'):
            self.integral_error = np.zeros((self.model.nq, 1))

        if controller_type == self.Controller.PD:
            # PD Controller
            tau = - self.Kp @ eq - self.Kd @ edq

        elif controller_type == self.Controller.PD_G:
            # PD + Gravity Compensation Controller
            q = np.array(q).flatten()
            G = self.model.G_fn(*q)#.astype(float)            
            tau = - self.Kp @ eq - self.Kd @ edq + G

        elif controller_type == self.Controller.PID:
            # PID Controller
            # accumulate integral error as done in modern robotics book
            q = np.array(q).flatten()
            G = self.model.G_fn(*q)#.astype(float)            
            self.integral_error += eq * dt
            tau = (- self.Kp @ eq 
                   - self.Kd @ edq 
                   - self.Ki @ self.integral_error)
            #when adding gravity the robot stays at the desired position
            # PID  with gravity compensation for positions that point upwards
            #tau = tau + G

        # TODO: Calculate Lyapunov function and publish
        q = np.asarray(q, dtype=float).flatten()
        M = self.model.M_fn(*q).astype(float)        
        M = M.reshape(self.model.nq, self.model.nq)
        print("qdot:", qdot, "edq:", edq)
        V = 0.5 * qdot.T @ M @ qdot + 0.5 * eq.T @ self.Kp @ eq
        lyapunov_msg = Float32()
        lyapunov_msg.data = float(V)
        self.lyapunov_pub.publish(lyapunov_msg)

        # TODO: Calculate steady state error and publish
        err = eq
        steady_state_error_msg = Float32MultiArray()
        steady_state_error_msg.data = err.flatten().tolist()
        self.steady_state_error_pub.publish(steady_state_error_msg)

        return tau
