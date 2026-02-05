"""
UR10 Joint Error Space Controller (3 DOF)

Passivity-based joint space controller implementing PD and PID control laws
with sinusoidal trajectory tracking across multiple states.
"""

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray
import numpy as np
from enum import Enum

from ur10_model.ur10_model_3dof import UR10Model3DOF


class UR10Controller_Joint_Error_Space:
    """Joint error space controller for UR10 robot using PD/PID laws."""

    class State(Enum):
        INIT = 1
        REGULATE = 2
        TRACK_SLOW = 3
        TRACK_FAST = 4
        END = 5

    class ControllerType(Enum):
        PD = 1
        PID = 2

    def __init__(self, model: UR10Model3DOF, q_init: np.ndarray = None) -> None:
        """
        Initialize controller.
        
        Args:
            model: UR10Model3DOF instance
            q_init: Initial joint configuration (default: zeros)
        """
        if q_init is None:
            q_init = np.zeros((3, 1))
            
        self.model = model
        self.q = q_init
        self.state = self.State.INIT
        self.prev_state = None
        self.controller_type = self.ControllerType.PD
        #self.controller_type = self.ControllerType.PID

        # Timing
        self.t_start = rospy.Time.now()
        self.t_start_traj = rospy.Time.now()

        # ROS Publishers
        self.joint_states_des_pub = rospy.Publisher(
            "/joint_states_des", JointState, queue_size=10
        )
        self.lyapunov_pub = rospy.Publisher(
            "/lyapunov", Float32, queue_size=10
        )
        self.steady_state_error_pub = rospy.Publisher(
            "/steady_state_error", Float32MultiArray, queue_size=10
        )

        # Control gains
        self.Kp = np.diag([15.0, 25.0, 20.0])
        self.Kd = np.diag([3.0, 3.0, 2.0])
        self.Ki = np.diag([4.0, 4.0, 2.0])

        # Integral error tracking with anti-windup
        self.integral_error = np.zeros((self.model.nq, 1))
        self.integral_error_max = 0.3

        # Trajectory parameters
        self.q_regulation = np.array([0.0, np.pi / 2, np.pi / 4]).reshape(3, 1)
        self.amplitude = np.pi / 2
        self.angular_velocity_slow = 0.4
        self.angular_velocity_fast = 0.8

        # State transition timings (seconds)
        self.duration_regulate = 60
        self.duration_track_slow = 60
        self.duration_track_fast = 60

    def update(self, joint_states: np.ndarray, t: rospy.Time, dt: rospy.Duration) -> np.ndarray:
        """
        Main control loop update.
        
        Args:
            joint_states: Current states [q, qdot, qddot] (3 x n)
            t: Current ROS time
            dt: Control loop timestep
            
        Returns:
            Torque commands (n x 1)
        """
        self.q = joint_states[0, :].reshape(-1, 1)
        t_elapsed = (t - self.t_start).to_sec()

        # State machine
        self._update_state(t_elapsed, t)

        # Desired trajectory
        joint_states_des = self._generate_trajectory(self.state, t)

        # Control law
        tau = self._compute_control(joint_states, joint_states_des, dt.to_sec())

        return tau

    def _update_state(self, t_elapsed: float, t_now: rospy.Time) -> None:
        """Update state machine based on elapsed time."""
        if self.state == self.State.INIT:
            self.state = self.State.REGULATE
            self.t_start = t_now
            self.t_start_traj = t_now
            return

        # Transition times
        t_regulate_end = self.duration_regulate
        t_track_slow_end = self.duration_regulate + self.duration_track_slow
        t_track_fast_end = t_track_slow_end + self.duration_track_fast

        # State transitions
        if t_elapsed < t_regulate_end:
            new_state = self.State.REGULATE
        elif t_elapsed < t_track_slow_end:
            new_state = self.State.TRACK_SLOW
        elif t_elapsed < t_track_fast_end:
            new_state = self.State.TRACK_FAST
        else:
            new_state = self.State.END

        # Reset trajectory timing on state transition
        if new_state != self.state:
            self.t_start_traj = t_now
            self.integral_error = np.zeros((self.model.nq, 1))
            
        self.state = new_state

    def _generate_trajectory(self, state: State, t: rospy.Time) -> np.ndarray:
        """
        Generate desired joint states for current state.
        
        Returns:
            Desired states [q_des, qdot_des, qddot_des] (3 x n)
        """
        # Default values
        q_des = np.zeros((self.model.nq, 1))
        qdot_des = np.zeros((self.model.nq, 1))
        qddot_des = np.zeros((self.model.nq, 1))

        if state == self.State.INIT:
            q_des = np.array([[0.0], [0.0], [0.1]])

        elif state == self.State.REGULATE:
            q_des = self.q_regulation.copy()

        elif state in [self.State.TRACK_SLOW, self.State.TRACK_FAST]:
            # Sinusoidal oscillation around regulation point
            w = self.angular_velocity_slow if state == self.State.TRACK_SLOW else self.angular_velocity_fast
            tau = (t - self.t_start_traj).to_sec()
            
            sin_term = np.sin(w * tau)
            cos_term = np.cos(w * tau)
            
            q_des = self.q_regulation + self.amplitude * np.array([[sin_term], [sin_term], [sin_term]])
            qdot_des = self.amplitude * w * np.array([[cos_term], [cos_term], [cos_term]])
            qddot_des = -self.amplitude * w**2 * np.array([[sin_term], [sin_term], [sin_term]])

        elif state == self.State.END:
            q_des = np.array([[np.pi / 4], [np.pi / 4], [np.pi / 4]])

        # Publish desired states
        self._publish_desired_states(q_des, qdot_des, qddot_des)

        return np.row_stack((q_des.flatten(), qdot_des.flatten(), qddot_des.flatten()))

    def _publish_desired_states(self, q_des: np.ndarray, qdot_des: np.ndarray, qddot_des: np.ndarray) -> None:
        """Publish desired joint states to ROS topic."""
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = [
            "ursa_shoulder_pan_joint",
            "ursa_shoulder_lift_joint",
            "ursa_elbow_joint",
            "ursa_wrist_1_joint",
            "ursa_wrist_2_joint",
            "ursa_wrist_3_joint"
        ]

        # Account for UR10 joint offsets
        msg.position = [
            float(q_des[0]),
            float(q_des[1] - np.pi / 2),  # shoulder_lift offset
            float(q_des[2]),
            -np.pi / 2,
            0.0,
            0.0
        ]
        msg.velocity = [
            float(qdot_des[0]),
            float(qdot_des[1]),
            float(qdot_des[2]),
            0.0,
            0.0,
            0.0
        ]
        msg.effort = [
            float(qddot_des[0]),
            float(qddot_des[1]),
            float(qddot_des[2]),
            0.0,
            0.0,
            0.0
        ]
        self.joint_states_des_pub.publish(msg)

    def _publish_lyapunov(self, V: float) -> None:
        """Publish Lyapunov function value."""
        msg = Float32()
        msg.data = float(V)
        self.lyapunov_pub.publish(msg)

    def _publish_steady_state_error(self, error: np.ndarray) -> None:
        """Publish steady-state error."""
        msg = Float32MultiArray()
        msg.data = error.flatten().tolist()
        self.steady_state_error_pub.publish(msg)

    def _compute_control(self, joint_states: np.ndarray, joint_states_des: np.ndarray, dt: float) -> np.ndarray:
        """
        Compute control torques using PD or PID law.
        
        Control law: τ = -Kd·S_q + M·q̈_r + C·q̇_r + G
        where S_q = q̇ - q̇_r (velocity error)
        """
        # Extract current and desired states
        q = joint_states[0, :].reshape(-1, 1)
        qdot = joint_states[1, :].reshape(-1, 1)

        q_des = joint_states_des[0, :].reshape(-1, 1)
        qdot_des = joint_states_des[1, :].reshape(-1, 1)
        qddot_des = joint_states_des[2, :].reshape(-1, 1)

        # Compute errors
        eq = q - q_des
        eqdot = qdot - qdot_des

        # Reference velocity computation
        if self.controller_type == self.ControllerType.PD:
            qdot_r = qdot_des - (self.Kp @ eq)
            qddot_r = qddot_des - (self.Kp @ eqdot)

        elif self.controller_type == self.ControllerType.PID:
            self.integral_error += eq * dt
            self.integral_error = np.clip(self.integral_error, -self.integral_error_max, self.integral_error_max)
            
            qdot_r = qdot_des - (self.Kp @ eq) - (self.Ki @ self.integral_error)
            qddot_r = qddot_des

        # Velocity error
        sq = qdot - qdot_r

        # Get dynamics matrices
        q_flat = np.asarray(q, dtype=float).flatten()
        qdot_flat = np.asarray(qdot, dtype=float).flatten()
        
        M = self.model.M_fn(*q_flat).astype(float).reshape(self.model.nq, self.model.nq)
        C = self.model.C_fn(*q_flat, *qdot_flat).astype(float).reshape(self.model.nq, self.model.nq)
        G = self.model.G_fn(*q_flat).astype(float).reshape(self.model.nq, 1)

        # Feedforward compensation
        Yr_theta_r= (M @ qddot_r) + (C @ qdot_r) + G

        # Control law
        tau = -(self.Kd @ sq) + Yr_theta_r

        # Debug logging
        self._log_control_info(q_flat, qdot_flat, qdot_r, qddot_r, sq, tau)

        self._publish_lyapunov(0.5 * sq.T @ M @ sq)

        self._publish_steady_state_error(eq)

        return tau

    def _log_control_info(self, q: np.ndarray, qdot: np.ndarray, qdot_r: np.ndarray, 
                         qddot_r: np.ndarray, sq: np.ndarray, tau: np.ndarray) -> None:
        """Log control values for debugging."""
        print(
            f"State: {self.state.name} | Controller: {self.controller_type.name}\n"
            f"|q|={np.linalg.norm(q):.4f}, |qdot|={np.linalg.norm(qdot):.4f}, "
            f"|qdot_r|={np.linalg.norm(qdot_r):.4f}, |qddot_r|={np.linalg.norm(qddot_r):.4f}, "
            f"|Sq|={np.linalg.norm(sq):.4f}, |tau|={np.linalg.norm(tau):.2f}"
        )