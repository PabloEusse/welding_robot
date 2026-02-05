"""
UR10 Operational Space Controller with PD/PID modes and singularity handling.

Task 3: Execute trajectory with two control modes (PD-like and PID-like)
  - A: Park, B: Non-singular config, C: Linear approach, D-E: Circular oscillations
Task 4: Singularity proximity experiment with larger circular trajectory
"""

import rospy
import numpy as np
from enum import Enum
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray


class ControlMode(Enum):
    DAMPING_PROPORTIONAL = 1
    INTEGRAL_DAMPING_PROPORTIONAL = 2


class ExecutionPhase(Enum):
    PARK_POSTURE = 0
    AVOID_SINGULAR_CONFIG = 1
    LINEAR_MOTION = 2
    CIRCULAR_OSCILLATION_SLOW = 3
    CIRCULAR_OSCILLATION_FAST = 4
    SEGMENT_COMPLETE = 5
    SINGULARITY_HOLD = 6
    SINGULARITY_CONTROL_DAMPING = 7
    SINGULARITY_CONTROL_INTEGRAL = 8
    SINGULARITY_RETURN_HOME = 9


class UR10OperationalSpaceController:
    """
    Operational-space controller: τ = -Kd(q̇ - q̇_r) + M*q̈_r + C*q̇_r + G
    """
    
    def __init__(self, model):
        self.model = model
        self.nq = int(model.nq)
        self.control_mode = ControlMode.DAMPING_PROPORTIONAL
        self.sequence_iteration = 0
        self.max_sequences = 2

        # Feedback gains
        self.damping_gain = np.diag([4.0, 4.0, 3.0])
        self.position_feedback_gain = np.diag([5.0, 5.0, 4.0])
        self.integral_feedback_gain = np.diag([4.0, 4.0, 4.0])

        # Integral error tracking and anti-windup
        self.accumulated_position_error = np.zeros((3, 1))
        self.error_accumulation_limit = 0.05

        # Phase timing (seconds)
        self.current_phase = ExecutionPhase.PARK_POSTURE
        self.phase_entry_time = rospy.Time.now()
        self.duration_park = 2.0
        self.duration_safe = 2.0
        self.duration_linear = 4.0
        self.duration_crown = 15.0

        # Joint configuration targets
        self.target_park_config = np.array([[0.0], [0.0], [0.25 * np.pi]])
        self.target_safe_config = np.array([[0.0], [0.0], [np.pi / 2.0]])
        self.target_home_config = self.target_park_config.copy()
        self.joint_level_gain = np.diag([2.0, 2.0, 1.5])

        # Cartesian trajectory parameters
        self.linear_shift_vector = np.array([[-0.3], [0.0], [-0.6]])
        self.circle_radius_nominal = 0.20
        self.vertical_oscillation_amplitude = 0.02
        self.angular_velocity_slow = 0.40
        self.angular_velocity_fast = 0.80

        # Computed during phase transitions
        self.end_effector_pose_at_b = None
        self.linear_start_pose = None
        self.orbital_center = None
        self.initial_angle = 0.0

        # Reference velocity differentiation
        self.previous_ref_velocity = np.zeros((self.nq, 1))
        self.ref_velocity_valid = False
        self.phase_transition_flag = True

        # Safety limits
        self.torque_limit = 120.0
        self.cartesian_velocity_limit = 0.25
        self.joint_velocity_limit = 2.0
        self.cartesian_velocity_limit_aggressive = 1.0
        self.joint_velocity_limit_aggressive = 4.0

        # Singularity handling (Task 4)
        self.singularity_orbit_center = None
        self.singularity_circle_radius = 0.55
        self.condition_number_threshold = 30
        self.singularity_encountered = False
        self.hold_duration = 4.0
        self.damping_active_duration = 6.0
        self.integral_active_duration = 6.0
        self.next_transition_target = None
        self.singularity_sequence_done = False

        self._setup_visualization()
        self.cartesian_error_publisher = rospy.Publisher(
            "/ur10/task_space_error", Float64MultiArray, queue_size=1
        )

    def _setup_visualization(self):
        """Initialize RViz trajectory markers with phase-dependent coloring."""
        self.trajectory_marker_publisher = rospy.Publisher(
            "/ur10/task3/desired_trajectory", Marker, queue_size=1
        )
        self._clear_rviz_display()

        self.trajectory_marker = Marker()
        self.trajectory_marker.header.frame_id = "world"
        self.trajectory_marker.ns = "task3_traj"
        self.trajectory_marker.id = 0
        self.trajectory_marker.type = Marker.LINE_STRIP
        self.trajectory_marker.action = Marker.ADD
        self.trajectory_marker.scale.x = 0.01
        self.trajectory_marker.color.a = 1.0
        self.trajectory_marker.pose.orientation.w = 1.0

        self.recorded_trajectory_points = []
        self._update_marker_color()
        rospy.on_shutdown(self._shutdown_visualization)

    def _clear_rviz_display(self):
        """Clear previous RViz markers."""
        clear_cmd = Marker()
        clear_cmd.action = Marker.DELETEALL
        clear_cmd.header.frame_id = "world"
        for _ in range(3):
            clear_cmd.header.stamp = rospy.Time.now()
            self.trajectory_marker_publisher.publish(clear_cmd)
            rospy.sleep(0.05)

    def _shutdown_visualization(self):
        """Clean up markers on shutdown."""
        rospy.loginfo("[Task] Clearing all RViz markers...")
        clear_cmd = Marker()
        clear_cmd.action = Marker.DELETEALL
        for _ in range(5):
            clear_cmd.header.stamp = rospy.Time.now()
            self.trajectory_marker_publisher.publish(clear_cmd)
            rospy.sleep(0.1)

    def _delete_current_marker(self):
        """Delete current marker from RViz."""
        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = rospy.Time.now()
        m.ns = self.trajectory_marker.ns
        m.id = self.trajectory_marker.id
        m.action = Marker.DELETE
        self.trajectory_marker_publisher.publish(m)

    def _start_new_trajectory_segment(self):
        """Start a new trajectory segment with fresh point list."""
        self.trajectory_marker.id += 1
        self.recorded_trajectory_points = []
        self.trajectory_marker.points = self.recorded_trajectory_points
        self.trajectory_marker.header.stamp = rospy.Time.now()

    def _update_marker_color(self):
        """Set marker color based on control mode and phase."""
        if self.current_phase == ExecutionPhase.SINGULARITY_CONTROL_DAMPING:
            self.trajectory_marker.color.r = 0.0
            self.trajectory_marker.color.g = 1.0
            self.trajectory_marker.color.b = 0.0
            return
        if self.current_phase == ExecutionPhase.SINGULARITY_CONTROL_INTEGRAL:
            self.trajectory_marker.color.r = 1.0
            self.trajectory_marker.color.g = 0.0
            self.trajectory_marker.color.b = 1.0
            return

        if self.control_mode == ControlMode.DAMPING_PROPORTIONAL:
            self.trajectory_marker.color.r = 0.0
            self.trajectory_marker.color.g = 0.0
            self.trajectory_marker.color.b = 1.0
        else:
            self.trajectory_marker.color.r = 1.0
            self.trajectory_marker.color.g = 0.0
            self.trajectory_marker.color.b = 0.0

    def _debug_jacobian_validation(self, q):
        """Verify Jacobian computation via numerical differentiation (optional)."""
        eps = 1e-6
        n = self.nq

        J6 = np.array(self.model.J_ee_w_fn(*q.flatten()), dtype=float)
        J_position = J6[0:3, :]
        J_orientation = J6[3:6, :]

        J_numerical = np.zeros((3, n))
        x0 = self._compute_ee_position(q)
        for i in range(n):
            q_plus = q.copy()
            q_minus = q.copy()
            q_plus[i, 0] += eps
            q_minus[i, 0] -= eps
            x_plus = self._compute_ee_position(q_plus)
            x_minus = self._compute_ee_position(q_minus)
            J_numerical[:, i] = ((x_plus - x_minus) / (2.0 * eps)).flatten()

        rospy.logwarn("====== Jacobian FULL check (position) ======")
        rospy.logwarn(f"J_num (pos, from FK) =\n{J_numerical}")
        rospy.logwarn(f"J6 position =\n{J_position}")
        rospy.logwarn(f"J6 orientation =\n{J_orientation}")

        for block_label, J_block in [("position", J_position), ("orientation", J_orientation)]:
            rospy.logwarn(f"---- Matching columns against {block_label} ----")
            for i in range(n):
                errors = [np.linalg.norm(J_numerical[:, i] - J_block[:, j]) for j in range(n)]
                best_match = int(np.argmin(errors))
                rospy.logwarn(
                    f"joint {i}: best match col {best_match} err={errors[best_match]:.4e} "
                    f"all={['%.2e' % e for e in errors]}"
                )

        if hasattr(self.model, "Jp_ee_w_fk_fn"):
            J_fk = np.array(self.model.Jp_ee_w_fk_fn(*q.flatten()), dtype=float)
            rospy.logwarn(f"Jp_fk (d FK pos / d q) =\n{J_fk}")
            rospy.logwarn(f"||J_num - Jp_fk|| = {np.linalg.norm(J_numerical - J_fk):.4e}")

    def _evaluate_singularity_proximity(self, q):
        """Check condition number to detect proximity to singularities."""
        J = self._get_linear_jacobian(q)
        condition_number = np.linalg.cond(J)

        rospy.loginfo_throttle(
            0.5,
            f"[Task4] cond(J)={condition_number:.2f} (th={self.condition_number_threshold:.2f})"
        )

        if condition_number > self.condition_number_threshold:
            if not self.singularity_encountered:
                rospy.logwarn(f"[Task4] Singularity detected! cond(J) = {condition_number:.2f}")
            self.singularity_encountered = True

        return condition_number

    def update(self, joint_states, t_now, dt_ros):
        """
        Main control loop: compute reference trajectory and joint torques.
        
        Returns: τ (control torque command)
        """
        q = joint_states[0, :].reshape(self.nq, 1)
        qdot = joint_states[1, :].reshape(self.nq, 1)

        if not hasattr(self, "_jacobian_validated"):
            self._jacobian_validated = True

        dt = float(dt_ros.to_sec())
        if dt <= 0.0 or not np.isfinite(dt):
            dt = 1e-3

        # Phase transitions
        self._handle_phase_transitions(t_now, q)
        
        # Generate reference velocity (q̇_r)
        qdot_ref = self._generate_reference_velocity(t_now, q, dt)

        # Avoid acceleration spikes at transitions
        if (not self.ref_velocity_valid) or self.phase_transition_flag:
            self.previous_ref_velocity = qdot_ref.copy()
            self.ref_velocity_valid = True
            self.phase_transition_flag = False

        # Differentiate reference velocity to get reference acceleration
        qddot_ref = (qdot_ref - self.previous_ref_velocity) / dt
        self.previous_ref_velocity = qdot_ref.copy()

        # Velocity error
        velocity_error = qdot - qdot_ref

        # Dynamics matrices for feedforward compensation
        M, C, G = self._get_dynamics_matrices(q, qdot)
        feedforward_term = (M @ qddot_ref) + (C @ qdot_ref) + G

        # Control law: τ = -Kd*S_q + Feedforward
        control_torque = (-self.damping_gain @ velocity_error) + feedforward_term
        control_torque = np.clip(control_torque, -self.torque_limit, self.torque_limit)

        # Visualization: record trajectory points
        if self.current_phase in [
            ExecutionPhase.LINEAR_MOTION,
            ExecutionPhase.CIRCULAR_OSCILLATION_SLOW,
            ExecutionPhase.CIRCULAR_OSCILLATION_FAST,
            ExecutionPhase.SINGULARITY_CONTROL_DAMPING,
            ExecutionPhase.SINGULARITY_CONTROL_INTEGRAL,
        ]:
            x_desired, _ = self._get_cartesian_reference(t_now)
            p = Point()
            p.x = float(x_desired[0, 0])
            p.y = float(x_desired[1, 0])
            p.z = float(x_desired[2, 0])
            self.recorded_trajectory_points.append(p)

            if len(self.recorded_trajectory_points) >= 2:
                self.trajectory_marker.points = self.recorded_trajectory_points
                self.trajectory_marker.header.frame_id = "world"
                self.trajectory_marker.header.stamp = rospy.Time.now()
                self.trajectory_marker_publisher.publish(self.trajectory_marker)

        # Logging
        task_label = "[Task4]" if self.current_phase in [
            ExecutionPhase.SINGULARITY_HOLD,
            ExecutionPhase.SINGULARITY_CONTROL_DAMPING,
            ExecutionPhase.SINGULARITY_CONTROL_INTEGRAL,
            ExecutionPhase.SINGULARITY_RETURN_HOME
        ] else "[Task3]"

        if self.current_phase in [ExecutionPhase.LINEAR_MOTION, ExecutionPhase.CIRCULAR_OSCILLATION_SLOW,
                                   ExecutionPhase.CIRCULAR_OSCILLATION_FAST,
                                   ExecutionPhase.SINGULARITY_CONTROL_DAMPING, ExecutionPhase.SINGULARITY_CONTROL_INTEGRAL]:
            x = self._compute_ee_position(q)
            xd, _ = self._get_cartesian_reference(t_now)
            dx = x - xd
            msg = Float64MultiArray()
            msg.data = dx.flatten().tolist()
            self.cartesian_error_publisher.publish(msg)
            rospy.loginfo_throttle(
                0.5,
                f"{task_label}[{self.control_mode.name}] phase={self.current_phase.name} "
                f"| ||dX||={np.linalg.norm(dx):.4f} m | ||Sv||={np.linalg.norm(velocity_error):.4f} "
                f"| ||tau||={np.linalg.norm(control_torque):.2f}"
            )
        else:
            q_target = self._get_target_joint_config()
            dq = q - q_target
            rospy.loginfo_throttle(
                0.5,
                f"{task_label} phase={self.current_phase.name} | ||dq||={np.linalg.norm(dq):.4f} rad "
                f"| ||Sv||={np.linalg.norm(velocity_error):.4f} | ||tau||={np.linalg.norm(control_torque):.2f}"
            )

        return control_torque

    def _handle_phase_transitions(self, t_now, q):
        """State machine: implement phase progression and transitions."""
        elapsed = (t_now - self.phase_entry_time).to_sec()

        # Task3: A -> B -> C -> D -> E -> DONE
        if self.current_phase == ExecutionPhase.PARK_POSTURE and elapsed >= self.duration_park:
            self._transition_to_phase(ExecutionPhase.AVOID_SINGULAR_CONFIG, t_now)
            return

        if self.current_phase == ExecutionPhase.AVOID_SINGULAR_CONFIG and elapsed >= self.duration_safe:
            self.end_effector_pose_at_b = self._compute_ee_position(q)
            self.linear_start_pose = self.end_effector_pose_at_b + self.linear_shift_vector
            self.orbital_center = self.linear_start_pose - np.array([[-self.circle_radius_nominal], [0.0], [0.0]])
            v0 = self.linear_start_pose - self.orbital_center
            self.initial_angle = np.arctan2(float(v0[1, 0]), float(v0[0, 0]))

            self._transition_to_phase(ExecutionPhase.LINEAR_MOTION, t_now)
            self._start_new_trajectory_segment()
            self._update_marker_color()
            return

        if self.current_phase == ExecutionPhase.LINEAR_MOTION and elapsed >= self.duration_linear:
            self._transition_to_phase(ExecutionPhase.CIRCULAR_OSCILLATION_SLOW, t_now)
            return

        if self.current_phase == ExecutionPhase.CIRCULAR_OSCILLATION_SLOW and elapsed >= self.duration_crown:
            self._transition_to_phase(ExecutionPhase.CIRCULAR_OSCILLATION_FAST, t_now)
            return

        if self.current_phase == ExecutionPhase.CIRCULAR_OSCILLATION_FAST and elapsed >= self.duration_crown:
            self._transition_to_phase(ExecutionPhase.SEGMENT_COMPLETE, t_now)
            return

        # Handle SEGMENT_COMPLETE: PD -> PID -> Task4
        if self.current_phase == ExecutionPhase.SEGMENT_COMPLETE:
            if self.singularity_sequence_done:
                return

            if self.sequence_iteration == 0:
                rospy.loginfo("[Task3] PD run completed. Restarting Task3 with PID-like controller.")
                self.sequence_iteration = 1
                self.control_mode = ControlMode.INTEGRAL_DAMPING_PROPORTIONAL
                self._transition_to_phase(ExecutionPhase.PARK_POSTURE, t_now)
                self._start_new_trajectory_segment()
                self._update_marker_color()
                return

            rospy.loginfo("[Task3] PID run completed. Entering Task4.")
            self._transition_to_phase(ExecutionPhase.SINGULARITY_HOLD, t_now)
            self.control_mode = ControlMode.DAMPING_PROPORTIONAL
            self.singularity_encountered = False
            self.next_transition_target = None
            return

        # Task4: WAIT -> PD -> RETURN -> PID -> RETURN -> DONE
        if self.current_phase == ExecutionPhase.SINGULARITY_HOLD:
            if elapsed >= self.hold_duration:
                rospy.loginfo("[Task4] Starting PD-like singularity test (forced duration).")
                self._transition_to_phase(ExecutionPhase.SINGULARITY_CONTROL_DAMPING, t_now)
                self.control_mode = ControlMode.DAMPING_PROPORTIONAL
                self.singularity_encountered = False
                self.next_transition_target = "to_pid"
                self._setup_singularity_trajectory(t_now)
                self._start_new_trajectory_segment()
                self._update_marker_color()
            return

        if self.current_phase == ExecutionPhase.SINGULARITY_CONTROL_DAMPING:
            if elapsed < self.damping_active_duration:
                return

            self._evaluate_singularity_proximity(q)
            if not self.singularity_encountered:
                rospy.logwarn(
                    f"[Task4] PD-like: near-singularity region reached "
                    f"(cond(J) below threshold {self.condition_number_threshold:.2f}). "
                    "Triggering protective return to preserve stability."
                )
                self.singularity_encountered = True

            rospy.logwarn("[Task4] PD-like: singularity handled -> returning home (joint control).")
            self._transition_to_phase(ExecutionPhase.SINGULARITY_RETURN_HOME, t_now)
            return

        if self.current_phase == ExecutionPhase.SINGULARITY_CONTROL_INTEGRAL:
            if elapsed < self.integral_active_duration:
                return

            self._evaluate_singularity_proximity(q)
            if not self.singularity_encountered:
                rospy.logwarn(
                    f"[Task4] PID-like: near-singularity region reached "
                    f"(cond(J) below threshold {self.condition_number_threshold:.2f}). "
                    "Triggering protective return to preserve stability."
                )
                self.singularity_encountered = True

            rospy.logwarn("[Task4] PID-like: singularity handled -> returning home (joint control).")
            self._transition_to_phase(ExecutionPhase.SINGULARITY_RETURN_HOME, t_now)
            return

        if self.current_phase == ExecutionPhase.SINGULARITY_RETURN_HOME:
            X = self._compute_ee_position(q)
            dX = X - self.linear_start_pose
            if np.linalg.norm(dX) < 0.03:
                if self.next_transition_target == "to_pid":
                    rospy.loginfo("[Task4] Reached home. Starting PID-like singularity test (forced duration).")
                    self._transition_to_phase(ExecutionPhase.SINGULARITY_CONTROL_INTEGRAL, t_now)
                    self.control_mode = ControlMode.INTEGRAL_DAMPING_PROPORTIONAL
                    self.accumulated_position_error[:] = 0.0
                    self.singularity_encountered = False
                    self.next_transition_target = "to_done"
                    self._start_new_trajectory_segment()
                    self._update_marker_color()
                    return

                if self.next_transition_target == "to_done":
                    rospy.loginfo("[Task4] Reached home. Task4 finished.")
                    self.singularity_sequence_done = True
                    self._transition_to_phase(ExecutionPhase.SEGMENT_COMPLETE, t_now)
            return

    def _transition_to_phase(self, new_phase, t_now):
        """Transition to new phase with reset of integrators and flags."""
        self.current_phase = new_phase
        self.phase_entry_time = t_now
        self._clear_error_accumulators()
        self.phase_transition_flag = True

    def _clear_error_accumulators(self):
        """Reset integral error accumulator."""
        self.accumulated_position_error[:] = 0.0

    def _generate_reference_velocity(self, t_now, q, dt):
        """Generate reference velocity based on current phase."""
        if self.current_phase == ExecutionPhase.SINGULARITY_RETURN_HOME:
            return self._compute_return_velocity(q)

        # Joint-space control for parking/safe config
        if self.current_phase in [ExecutionPhase.PARK_POSTURE, ExecutionPhase.AVOID_SINGULAR_CONFIG]:
            target = self.target_park_config if self.current_phase == ExecutionPhase.PARK_POSTURE else self.target_safe_config
            return self._compute_joint_reaching_velocity(q, target)

        # Operational-space control: x̌̇_r -> qdot_r via Jacobian
        xdot_ref = self._compute_cartesian_reference_velocity(t_now, q, dt)
        J = self._get_linear_jacobian(q)
        qdot_ref = np.linalg.pinv(J) @ xdot_ref

        if self.current_phase in [ExecutionPhase.SINGULARITY_CONTROL_DAMPING, ExecutionPhase.SINGULARITY_CONTROL_INTEGRAL]:
            qdot_ref = np.clip(qdot_ref, -self.joint_velocity_limit_aggressive, self.joint_velocity_limit_aggressive)
        else:
            qdot_ref = np.clip(qdot_ref, -self.joint_velocity_limit, self.joint_velocity_limit)

        return qdot_ref

    def _compute_joint_reaching_velocity(self, q, q_target):
        """Simple joint-space control to reach target configuration."""
        dq = q - q_target
        qdot_ref = -(self.joint_level_gain @ dq)
        qdot_ref = np.clip(qdot_ref, -self.joint_velocity_limit, self.joint_velocity_limit)
        return qdot_ref

    def _compute_cartesian_reference_velocity(self, t_now, q, dt):
        """Compute reference EE velocity with PD/PID feedback on position error."""
        x_desired, xdot_desired = self._get_cartesian_reference(t_now)
        x_actual = self._compute_ee_position(q)
        position_error = x_actual - x_desired

        # Proportional feedback
        if self.current_phase in [ExecutionPhase.SINGULARITY_CONTROL_DAMPING, ExecutionPhase.SINGULARITY_CONTROL_INTEGRAL]:
            xdot_ref = xdot_desired - (0.5 * self.position_feedback_gain @ position_error)
        else:
            xdot_ref = xdot_desired - (self.position_feedback_gain @ position_error)

        # Integral feedback (PID mode only)
        if self.control_mode == ControlMode.INTEGRAL_DAMPING_PROPORTIONAL:
            self.accumulated_position_error += position_error * dt
            self.accumulated_position_error = np.clip(self.accumulated_position_error, -0.02, 0.02)
            xdot_ref -= (0.3 * self.integral_feedback_gain @ self.accumulated_position_error)

        # Velocity saturation
        if self.current_phase in [ExecutionPhase.SINGULARITY_CONTROL_DAMPING, ExecutionPhase.SINGULARITY_CONTROL_INTEGRAL]:
            xdot_ref = np.clip(xdot_ref, -self.cartesian_velocity_limit_aggressive, self.cartesian_velocity_limit_aggressive)
        else:
            xdot_ref = np.clip(xdot_ref, -self.cartesian_velocity_limit, self.cartesian_velocity_limit)

        return xdot_ref

    def _get_cartesian_reference(self, t_now):
        """Compute desired EE position and velocity for current phase."""
        tau = (t_now - self.phase_entry_time).to_sec()

        if self.end_effector_pose_at_b is None or self.linear_start_pose is None or self.orbital_center is None:
            x_hold = self._compute_ee_position(self.target_safe_config)
            return x_hold, np.zeros((3, 1))

        # Linear interpolation from B to C
        if self.current_phase == ExecutionPhase.LINEAR_MOTION:
            T = self.duration_linear
            s = np.clip(tau / T, 0.0, 1.0)
            x_desired = (1.0 - s) * self.end_effector_pose_at_b + s * self.linear_start_pose
            xdot_desired = (self.linear_start_pose - self.end_effector_pose_at_b) / T if tau < T else np.zeros((3, 1))
            return x_desired, xdot_desired

        # Circular motion with vertical oscillation (Task3)
        if self.current_phase in [ExecutionPhase.CIRCULAR_OSCILLATION_SLOW, ExecutionPhase.CIRCULAR_OSCILLATION_FAST]:
            w = self.angular_velocity_slow if self.current_phase == ExecutionPhase.CIRCULAR_OSCILLATION_SLOW else self.angular_velocity_fast
            theta = self.initial_angle + w * tau
            c = self.orbital_center
            k = 10.0

            x_desired = np.array([
                [c[0, 0] + self.circle_radius_nominal * np.cos(theta)],
                [c[1, 0] + self.circle_radius_nominal * np.sin(theta)],
                [c[2, 0] + self.vertical_oscillation_amplitude * np.sin(k * theta)],
            ])

            xdot_desired = np.array([
                [-self.circle_radius_nominal * w * np.sin(theta)],
                [self.circle_radius_nominal * w * np.cos(theta)],
                [self.vertical_oscillation_amplitude * k * w * np.cos(k * theta)],
            ])
            return x_desired, xdot_desired

        # Large circle toward singularity (Task4)
        if self.current_phase in [ExecutionPhase.SINGULARITY_CONTROL_DAMPING, ExecutionPhase.SINGULARITY_CONTROL_INTEGRAL]:
            w = self.angular_velocity_slow
            theta = self.initial_angle + w * tau
            c = self.singularity_orbit_center
            k = 10.0

            x_desired = np.array([
                [c[0, 0] + self.singularity_circle_radius * np.cos(theta)],
                [c[1, 0] + self.singularity_circle_radius * np.sin(theta)],
                [c[2, 0] + self.vertical_oscillation_amplitude * np.sin(k * theta)],
            ])

            xdot_desired = np.array([
                [-self.singularity_circle_radius * w * np.sin(theta)],
                [self.singularity_circle_radius * w * np.cos(theta)],
                [self.vertical_oscillation_amplitude * k * w * np.cos(k * theta)],
            ])
            return x_desired, xdot_desired

        return self.linear_start_pose.copy(), np.zeros((3, 1))

    def _setup_singularity_trajectory(self, t_now):
        """Compute circle center for large trajectory toward singularity."""
        k = 10.0
        c4 = self.linear_start_pose.copy()
        c4[0, 0] -= self.singularity_circle_radius * np.cos(self.initial_angle)
        c4[1, 0] -= self.singularity_circle_radius * np.sin(self.initial_angle)
        c4[2, 0] -= self.vertical_oscillation_amplitude * np.sin(k * self.initial_angle)
        self.singularity_orbit_center = c4

    def _compute_return_velocity(self, q):
        """Joint velocity to return to starting position using operational-space control."""
        X = self._compute_ee_position(q)
        dX = X - self.linear_start_pose
        xdot_r = -(self.position_feedback_gain @ dX)
        J = self._get_linear_jacobian(q)
        qdot_r = np.linalg.pinv(J) @ xdot_r
        return np.clip(qdot_r, -self.joint_velocity_limit, self.joint_velocity_limit)

    def _compute_ee_position(self, q):
        """Extract EE position from forward kinematics transform."""
        T = np.array(self.model.T_ee_w_fn(*q.flatten()), dtype=float)
        return T[0:3, 3].reshape(3, 1)

    def _get_linear_jacobian(self, q):
        """Get position Jacobian (top 3 rows of full Jacobian)."""
        if not hasattr(self.model, "J_ee_w_fn"):
            raise RuntimeError("Model does not provide J_ee_w_fn. Check ur10_model_3dof.py jacobians().")
        Jp = np.array(self.model.J_ee_w_fn(*q.flatten()), dtype=float)[0:3, :]
        return Jp

    def _get_dynamics_matrices(self, q, qdot):
        """Get inertia M, Coriolis C, and gravity G matrices for feedforward."""
        n = self.nq
        q = np.asarray(q, dtype=float).flatten()
        M = self.model.M_fn(*q).astype(float).reshape(self.model.nq, self.model.nq)
        qdot = np.asarray(qdot, dtype=float).flatten()
        C = self.model.C_fn(*q, *qdot).astype(float).reshape(self.model.nq, self.model.nq)
        G = self.model.G_fn(*q).astype(float).reshape(self.model.nq, 1)
        return M, C, G

    def _get_target_joint_config(self):
        """Return target configuration for current phase."""
        if self.current_phase == ExecutionPhase.PARK_POSTURE:
            return self.target_park_config
        if self.current_phase == ExecutionPhase.AVOID_SINGULAR_CONFIG:
            return self.target_safe_config
        if self.current_phase == ExecutionPhase.SINGULARITY_RETURN_HOME:
            return self.target_home_config
        return self.target_safe_config