import rospy
import numpy as np
import sympy as sym
from enum import Enum
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray


class UR10OperationalSpaceController:
    class Mode(Enum):
        JP_PD = 1
        JP_PID = 2
        OP_PD = 3
        OP_PID = 4


    class Phase(Enum):
        # ---------- Task 3 (Operational Space) ----------
        A_PARK = 0
        B_NON_SINGULAR = 1
        C_LINEAR = 2
        D_CROWN_W1 = 3
        E_CROWN_W2 = 4
        DONE = 5

        # ---------- Joint Space Control (Adaptive) ----------
        JS_REGULATE = 6       # Regulation: constant q_d
        JS_TRACK_SLOW = 7     # Tracking: sinusoidal with w1
        JS_TRACK_FAST = 8     # Tracking: sinusoidal with w2
        JS_DONE = 9

    def __init__(self, model):
        self.model = model
        self.nq = int(model.nq)

        # -------------------------------------------------
        # Controller mode (PD-like vs PID-like)
        # -------------------------------------------------
        self.mode = self.Mode.OP_PD

        # -------------------------------------------------
        # Experiment sequence: JP_PD → JP_PID → OP_PD → OP_PID
        # -------------------------------------------------
        # Task3 (operational space) experiment loop: run_id = 0 (PD), 1 (PID)
        self.run_id = 0
        self.max_runs = 2

        # -------------------------------------------------
        # Gains (document-consistent tuning)
        # -------------------------------------------------
        # Kd: damping on Sq
        self.Kd = np.diag([4.0, 4.0, 3.0])       # damping on Sq in (2.1)
        self.Kp_x = np.diag([5.0, 5.0, 4.0])     # [1/s]
        self.Ki_x = np.diag([4.0, 4.0, 4.0])     # [1/s^2], Kp^2 - 4Ki <= 0 per axis

        # Integral anti-windup for ∫ΔX (operational space)
        self.int_x = np.zeros((3, 1))
        self.int_x_max = 0.05  # meters

        # -------------------------------------------------
        # Joint-Space Gains (for JP_PD, JP_PID modes)
        # -------------------------------------------------
        # NOTE: These gains multiply position error to get reference velocity
        # Too high = oscillations/zig-zag. Keep similar magnitude to Kp_x.
        self.Kp_js = np.diag([4.0, 4.0, 3.0])     # Position gain for joint space (reduced from [15, 25, 20])
        self.Ki_js = np.diag([1.0, 1.0, 0.5])     # Integral gain for joint space (reduced from [4, 4, 2])
        
        # Integral anti-windup for ∫Δq (joint space)
        self.int_q = np.zeros((self.nq, 1))
        self.int_q_max = 0.3  # radians

        # -------------------------------------------------
        # Phase timing - Start with Joint-Space experiment
        # Sequence: JP_PD → JP_PID → OP_PD → OP_PID
        # -------------------------------------------------
        self.phase = self.Phase.JS_REGULATE
        self.mode = self.Mode.JP_PD
        self.js_experiment_active = True
        rospy.loginfo("[Experiment] Starting with Joint-Space JP_PD")
        rospy.loginfo("[Experiment] Sequence: JP_PD → JP_PID → OP_PD → OP_PID")
        self.phase_start = rospy.Time.now()

        self.T_a = 2.0
        self.T_b = 2.0
        self.T_c = 4.0
        self.T_d = 15.0

        self.T_a = 5.0
        self.T_b = 5.0
        self.T_c = 5.0
        self.T_d = 20.0

        self.js_run_id = 0   # 0 = JP_PD, 1 = JP_PID
        # -------------------------------------------------
        # Joint targets
        # -------------------------------------------------
        self.q_park = np.array([[0.0], [0.0], [0.25 * np.pi]])
        self.q_safe = np.array([[0.0], [0.0], [np.pi / 2.0]])
        self.Kp_q = np.diag([2.0, 2.0, 1.5])  # Joint motion gain for phases A, B

        # -------------------------------------------------
        # Joint-Space Trajectory Parameters (for JS phases)
        # -------------------------------------------------
        self.q_regulation = np.array([[0.0], [np.pi / 2], [np.pi / 4]])  # Regulation target
        self.q_amplitude = np.pi / 4                                       # Sinusoidal amplitude (reduced from π/2)
        self.w_js_slow = 0.3                                               # Slow tracking frequency [rad/s] (reduced from 0.4)
        self.w_js_fast = 0.6                                               # Fast tracking frequency [rad/s] (reduced from 0.8)
        
        # Joint-space phase durations (seconds)
        self.duration_js_regulate = 5.0
        self.duration_js_track_slow = 20.0
        self.duration_js_track_fast = 20.0

        
        # -------------------------------------------------
        # Cartesian trajectory parameters (Task3 c,d,e)
        # -------------------------------------------------
        self.linear_offset = np.array([[-0.3], [0.0], [-0.6]])

        self.radius = 0.20
        self.z_amp = 0.02
        self.w1 = 0.40
        self.w2 = 0.80

        self.x_b = None
        self.x_c_start = None
        self.circle_center = None
        self.theta0 = 0.0

        # -------------------------------------------------
        # qdot_r differentiation (for qddot_r)
        # -------------------------------------------------
        self.qdot_r_prev = np.zeros((self.nq, 1))
        self._qdot_r_prev_valid = False
        self._just_switched_phase = True

        # -------------------------------------------------
        # Saturations
        # -------------------------------------------------
        self.tau_max = 120.0
        self.xdot_r_max = 0.25
        self.qdot_r_max = 2.0
        self.qdot_r_max_js = 1.0  # Lower limit for joint-space control phases
        self.qddot_r_max = 10.0

        # -------------------------------------------------
        # RViz trajectory visualization (Marker)
        # -------------------------------------------------
        self.traj_pub = rospy.Publisher(
            "/ur10/task3/desired_trajectory",
            Marker,
            queue_size=1
        )

        # Clear old markers (best-effort; RViz subscription timing can vary)
        self._publish_deleteall_markers()

        self.traj_marker = Marker()
        self.traj_marker.header.frame_id = "world"
        self.traj_marker.ns = "task3_traj"
        self.traj_marker.id = 0
        self.traj_marker.type = Marker.LINE_STRIP
        self.traj_marker.action = Marker.ADD
        self.traj_marker.scale.x = 0.01
        self.traj_marker.color.a = 1.0
        self.traj_marker.pose.orientation.w = 1.0

        self.traj_points = []
        self._set_marker_color_for_current_run()

        rospy.on_shutdown(self._clear_all_markers)

        self.pub_task_space_error = rospy.Publisher(
            "/ur10/task_space_error",
            Float64MultiArray,
            queue_size=1
        )

        self.pub_joint_space_error = rospy.Publisher(
            "/ur10/joint_space_error",
            Float64MultiArray,
            queue_size=1
        )
        
        # -------------------------------------------------
        # ADAPTIVE CONTROL: Parameter estimation
        # τ = -Kd * Sq + Yr * Θ̂
        # Θ̂_dot = -Γ^(-1) * Yr^T * Sq
        # -------------------------------------------------
        self._setup_adaptive_control()

        self.pub_param_error = rospy.Publisher(
            "/ur10/parameter_error",
            Float64MultiArray,
            queue_size=1
        )

        self.pub_lyapunov = rospy.Publisher(
            "/ur10/lyapunov",
            Float64MultiArray,
            queue_size=1
        )

        self.pub_steady_state_error = rospy.Publisher(
            "/ur10/steady_state_error",
            Float64MultiArray,
            queue_size=1
        )

    # =========================================================
    # Adaptive Control Setup
    # =========================================================
    def _setup_adaptive_control(self):
        """
        Initialize adaptive control components:
        - Regressor matrix Yr (lambdified function)
        - True parameter vector theta_r (for comparison)
        - Estimated parameter vector theta_hat
        - Adaptation gain matrix Gamma
        """
        # Get symbolic regressor from model
        self.Yr_sym = self.model.Yr
        self.theta_r_sym = self.model.theta_r
        
        # Number of parameters
        self.num_params = self.theta_r_sym.shape[0]
        rospy.loginfo(f"[Adaptive] Number of parameters: {self.num_params}")
        
        # Create lambdified function for Yr
        # Yr depends on: q, qdot, qdot_r, qddot_r
        n = self.nq
        q_syms = sym.symbols('q(0:'+str(n)+')')
        qdot_syms = sym.symbols('q(0:'+str(n)+')dot')
        qdot_r_syms = sym.symbols('qr(0:'+str(n)+')dot')
        qddot_r_syms = sym.symbols('qr(0:'+str(n)+')ddot')
        
        self.Yr_fn = sym.lambdify(
            (q_syms, qdot_syms, qdot_r_syms, qddot_r_syms),
            self.Yr_sym,
            modules='numpy'
        )
        
        # Get true parameter values (for comparison/logging only)
        theta_fn = sym.lambdify((), self.theta_r_sym, 'numpy')
        self.theta_true = np.asarray(theta_fn(), dtype=float).reshape(-1, 1)
        rospy.loginfo(f"[Adaptive] True parameters shape: {self.theta_true.shape}")
        
        # Initialize estimated parameters with perturbation (simulating uncertainty)
        # Option 1: Start with perturbed true values (partial uncertainty)
        # perturbation = 0.1  # 10% perturbation (reduced from 30%)
        # self.theta_hat = self.theta_true * (1.0 + perturbation * (2.0 * np.random.rand(self.num_params, 1) - 1.0))
        
        # Option 2: Start with zeros (full uncertainty) - uncomment to use
        #self.theta_hat = np.zeros((self.num_params, 1))
        
        # Option 3: Start with true values (no uncertainty) - for testing stability
        # THIS SHOULD BEHAVE LIKE THE ORIGINAL REGRESSOR-BASED CONTROLLER
        self.theta_hat = self.theta_true.copy()
        
        # Adaptation gain matrix Γ (positive definite)
        # CRITICAL: Too high = oscillations/zig-zag, too low = slow adaptation
        self.gamma_value = 0.5  # Moderate adaptation gain
        self.gamma_value_fast = 0.0  # Disable adaptation during fast tracking (prevents divergence)
        self.Gamma = self.gamma_value * np.eye(self.num_params)
        self.Gamma_inv = np.eye(self.num_params) if self.gamma_value == 0 else np.linalg.inv(self.Gamma)
        
        # Parameter projection bounds: prevent theta_hat from drifting too far
        # Allow theta_hat to be within [0.5 * theta_true, 2.0 * theta_true] element-wise
        self.theta_min = 0.2 * self.theta_true  # Lower bound
        self.theta_max = 3.0 * self.theta_true  # Upper bound
        # Handle negative and zero parameters
        for i in range(self.num_params):
            if self.theta_true[i, 0] < 0:
                self.theta_min[i, 0], self.theta_max[i, 0] = self.theta_max[i, 0], self.theta_min[i, 0]
            elif abs(self.theta_true[i, 0]) < 1e-6:
                self.theta_min[i, 0] = -10.0
                self.theta_max[i, 0] = 10.0
        
        rospy.loginfo(f"[Adaptive] Initial parameter error norm: {np.linalg.norm(self.theta_hat - self.theta_true):.4f}")

    def _compute_regressor(self, q, qdot, qdot_r, qddot_r):
        """
        Compute the regressor matrix Yr at current state.
        Yr * theta = M(q) * qddot_r + C(q, qdot) * qdot_r + G(q)
        
        Returns: Yr (nq x num_params)
        """
        q_flat = np.asarray(q, dtype=float).flatten()
        qdot_flat = np.asarray(qdot, dtype=float).flatten()
        qdot_r_flat = np.asarray(qdot_r, dtype=float).flatten()
        qddot_r_flat = np.asarray(qddot_r, dtype=float).flatten()
        
        Yr = np.asarray(
            self.Yr_fn(q_flat, qdot_flat, qdot_r_flat, qddot_r_flat),
            dtype=float
        )
        
        return Yr

    def _compute_lyapunov(self, q, Sq):
        """
        Compute the Lyapunov function for adaptive control:
            V = (1/2) * Sq^T * M(q) * Sq + (1/2) * ΔΘ^T * Γ * ΔΘ
        
        where:
            - Sq = q̇ - q̇_r (sliding variable)
            - M(q) is the mass/inertia matrix
            - ΔΘ = θ̂ - θ_true (parameter estimation error)
            - Γ is the adaptation gain matrix
        
        Returns: V (scalar), V_kinetic (first term), V_param (second term)
        """
        # Ensure correct shapes
        Sq = Sq.reshape(self.nq, 1)
        q_flat = np.asarray(q, dtype=float).flatten()
        
        # Get mass matrix M(q)
        M = np.array(self.model.M_fn(*q_flat), dtype=float)
        
        # Parameter estimation error: ΔΘ = θ̂ - θ_true
        delta_theta = self.theta_hat - self.theta_true
        
        # Kinetic energy term: (1/2) * Sq^T * M(q) * Sq
        V_kinetic = 0.5 * float(Sq.T @ M @ Sq)
        
        # Parameter error term: (1/2) * ΔΘ^T * Γ * ΔΘ
        V_param = 0.5 * float(delta_theta.T @ self.Gamma @ delta_theta)
        
        # Total Lyapunov function
        V = V_kinetic + V_param
        
        return V, V_kinetic, V_param

    def _publish_lyapunov_and_error(self, q, Sq, t_now):
        """
        Publish Lyapunov function value and steady-state error.
        """
        # Compute Lyapunov function
        V, V_kinetic, V_param = self._compute_lyapunov(q, Sq)
        
        # Publish Lyapunov function: [V_total, V_kinetic, V_param]
        lyap_msg = Float64MultiArray()
        lyap_msg.data = [V, V_kinetic, V_param]
        self.pub_lyapunov.publish(lyap_msg)
        
        # Compute and publish steady-state error based on current phase
        error_msg = Float64MultiArray()
        
        if self.phase in [self.Phase.JS_REGULATE, self.Phase.JS_TRACK_SLOW, 
                          self.Phase.JS_TRACK_FAST, self.Phase.JS_DONE]:
            # Joint-space: steady-state error is joint position error
            q_d, _, _ = self._joint_space_trajectory(t_now)
            dq = q - q_d
            error_norm = float(np.linalg.norm(dq))
            # [error_norm, eq1, eq2, eq3]
            error_msg.data = [error_norm] + dq.flatten().tolist()
            
        elif self.phase in [self.Phase.C_LINEAR, self.Phase.D_CROWN_W1, self.Phase.E_CROWN_W2]:
            # Operational-space: steady-state error is Cartesian position error
            x = self._fk_pos(q)
            xd, _ = self._cartesian_desired(t_now)
            dx = x - xd
            error_norm = float(np.linalg.norm(dx))
            # [error_norm, ex, ey, ez]
            error_msg.data = [error_norm] + dx.flatten().tolist()
            
        else:
            # Preparatory phases: joint error to target
            q_target = self._current_joint_target()
            dq = q - q_target
            error_norm = float(np.linalg.norm(dq))
            error_msg.data = [error_norm] + dq.flatten().tolist()
        
        self.pub_steady_state_error.publish(error_msg)
        
        return V, V_kinetic, V_param

    def _update_parameter_estimates(self, Yr, Sq, dt):
        """
        Update parameter estimates using gradient descent:
            Θ̂_dot = -Γ^(-1) * Yr^T * Sq
            Θ̂(k+1) = Θ̂(k) + Θ̂_dot * dt
        
        Includes parameter projection to prevent divergence.
        """
        # Ensure correct shapes
        Sq = Sq.reshape(self.nq, 1)
        
        # Check if we should disable/reduce adaptation for fast tracking phases
        if self.phase == self.Phase.JS_TRACK_FAST:
            # Use reduced/zero gamma for fast tracking to prevent divergence
            if self.gamma_value_fast == 0:
                return np.zeros((self.num_params, 1))  # No adaptation
            Gamma_inv_use = (1.0 / self.gamma_value_fast) * np.eye(self.num_params)
        else:
            Gamma_inv_use = self.Gamma_inv
        
        # Gradient descent update law (eq 2.2)
        theta_hat_dot = -Gamma_inv_use @ (Yr.T @ Sq)
        
        # Clamp update rate to prevent explosive growth
        max_update_rate = 5.0  # Maximum parameter change rate per second
        theta_hat_dot = np.clip(theta_hat_dot, -max_update_rate, max_update_rate)
        
        # Euler integration
        self.theta_hat = self.theta_hat + theta_hat_dot * dt
        
        # Parameter projection: keep theta_hat within reasonable bounds
        self.theta_hat = np.clip(self.theta_hat, self.theta_min, self.theta_max)
        
        return theta_hat_dot

    # =========================================================
    # RViz helpers
    # =========================================================
    def _publish_deleteall_markers(self):
        clear = Marker()
        clear.action = Marker.DELETEALL
        clear.header.frame_id = "world"
        for _ in range(3):
            clear.header.stamp = rospy.Time.now()
            self.traj_pub.publish(clear)
            rospy.sleep(0.05)

    def _clear_all_markers(self):
        rospy.loginfo("[Task] Clearing all RViz markers...")
        clear = Marker()
        clear.action = Marker.DELETEALL
        for _ in range(5):
            clear.header.stamp = rospy.Time.now()
            self.traj_pub.publish(clear)
            rospy.sleep(0.1)

    def _reset_marker_new_segment(self):
        # RViz distinguishes markers by (ns, id). Bump id to start a new polyline object.
        self.traj_marker.id += 1
        self.traj_points = []
        self.traj_marker.points = self.traj_points
        self.traj_marker.header.stamp = rospy.Time.now()

    def _set_marker_color_for_current_run(self):
        # Task3 PD run: blue, Task3 PID run: red
        if self.mode == self.Mode.OP_PD:
            self.traj_marker.color.r = 0.0
            self.traj_marker.color.g = 0.0
            self.traj_marker.color.b = 1.0
        else:
            self.traj_marker.color.r = 1.0
            self.traj_marker.color.g = 0.0
            self.traj_marker.color.b = 0.0

    # =========================================================
    # Main update
    # =========================================================
    def update(self, joint_states, t_now, dt_ros):
        q = joint_states[0, :].reshape(self.nq, 1)
        qdot = joint_states[1, :].reshape(self.nq, 1)

        dt = float(dt_ros.to_sec())
        if dt <= 0.0 or not np.isfinite(dt):
            dt = 1e-3

        # Phase machine
        self._phase_switch(t_now, q)

        # Compute qdot_r
        qdot_r = self._compute_qdot_r(t_now, q, dt)

        # Avoid qddot_r spike on first step and right after phase switch
        if (not self._qdot_r_prev_valid) or self._just_switched_phase:
            self.qdot_r_prev = qdot_r.copy()
            self._qdot_r_prev_valid = True
            self._just_switched_phase = False

        qddot_r = (qdot_r - self.qdot_r_prev) / dt
        qddot_r = np.clip(qddot_r, -self.qddot_r_max, self.qddot_r_max)  # Clamp to prevent spikes from numerical differentiation
        self.qdot_r_prev = qdot_r.copy()

        Sq = qdot - qdot_r

        # ========== ADAPTIVE CONTROL LAW ==========
        # Compute regressor matrix Yr(q, qdot, qdot_r, qddot_r)
        Yr = self._compute_regressor(q, qdot, qdot_r, qddot_r)
        
        # Update parameter estimates: Θ̂_dot = -Γ^(-1) * Yr^T * Sq (eq 2.2)
        self._update_parameter_estimates(Yr, Sq, dt)
        
        # Compute feedforward term using estimated parameters: Yr * Θ̂
        Yr_theta_hat = Yr @ self.theta_hat
        
        # Control law: τ = -Kd * Sq + Yr * Θ̂ (eq 2.1)
        tau = (-self.Kd @ Sq) + Yr_theta_hat
        tau = np.clip(tau, -self.tau_max, self.tau_max)
        
        # Compute parameter estimation error for logging
        param_error_norm = np.linalg.norm(self.theta_hat - self.theta_true)

        # Publish Lyapunov function and steady-state error
        V, V_kinetic, V_param = self._publish_lyapunov_and_error(q, Sq, t_now)

        # Publish RViz trajectory markers for operational-space phases
        if self.phase in [
            self.Phase.C_LINEAR,
            self.Phase.D_CROWN_W1,
            self.Phase.E_CROWN_W2,
        ]:
            Xd, _ = self._cartesian_desired(t_now)

            p = Point()
            p.x = float(Xd[0, 0])
            p.y = float(Xd[1, 0])
            p.z = float(Xd[2, 0])

            self.traj_points.append(p)

            # LINE_STRIP requires at least 2 points
            if len(self.traj_points) >= 2:
                self.traj_marker.points = self.traj_points
                self.traj_marker.header.frame_id = "world"
                self.traj_marker.header.stamp = rospy.Time.now()
                self.traj_pub.publish(self.traj_marker)

        # Publish parameter error
        param_msg = Float64MultiArray()
        param_msg.data = [param_error_norm]
        self.pub_param_error.publish(param_msg)

        # Logging
        tag = "[Adaptive]"

        if self.phase in [self.Phase.C_LINEAR, self.Phase.D_CROWN_W1, self.Phase.E_CROWN_W2]:
            x = self._fk_pos(q)
            xd, _ = self._cartesian_desired(t_now)
            dx = x - xd
            # -------------------------------------------------
            # Task 5: publish Cartesian steady-state / tracking error
            # e_x = x - x_d
            # -------------------------------------------------
            msg = Float64MultiArray()
            msg.data = dx.flatten().tolist()   # [ex, ey, ez]
            self.pub_task_space_error.publish(msg)
            rospy.loginfo_throttle(
                0.5,
                f"{tag}[{self.mode.name}] phase={self.phase.name} "
                f"| ||dX||={np.linalg.norm(dx):.4f} m | ||Sq||={np.linalg.norm(Sq):.4f} "
                f"| ||dΘ||={param_error_norm:.4f} | V={V:.4f} | ||τ||={np.linalg.norm(tau):.2f}"
            )
        elif self.phase in [self.Phase.JS_REGULATE, self.Phase.JS_TRACK_SLOW, 
                            self.Phase.JS_TRACK_FAST, self.Phase.JS_DONE]:
            # Joint-space control phases - log joint error
            q_d, qdot_d, _ = self._joint_space_trajectory(t_now)
            dq = q - q_d
            
            # Publish joint-space error
            msg = Float64MultiArray()
            msg.data = dq.flatten().tolist()  # [eq1, eq2, eq3]
            self.pub_joint_space_error.publish(msg)
            
            rospy.loginfo_throttle(
                0.5,
                f"{tag}[{self.mode.name}] phase={self.phase.name} "
                f"| ||dq||={np.linalg.norm(dq):.4f} rad | ||Sq||={np.linalg.norm(Sq):.4f} "
                f"| ||dΘ||={param_error_norm:.4f} | V={V:.4f} | ||τ||={np.linalg.norm(tau):.2f}"
            )
        else:
            q_target = self._current_joint_target()
            dq = q - q_target
            rospy.loginfo_throttle(
                0.5,
                f"{tag} phase={self.phase.name} | ||dq||={np.linalg.norm(dq):.4f} rad "
                f"| ||Sq||={np.linalg.norm(Sq):.4f} | ||dΘ||={param_error_norm:.4f} | V={V:.4f} | ||τ||={np.linalg.norm(tau):.2f}"
            )

        return tau

    # =========================================================
    # Phase switching logic (implements your required flow)
    # =========================================================
    def _phase_switch(self, t_now, q):
        elapsed = (t_now - self.phase_start).to_sec()

        # -------------------------------------------------
        # Task3: A -> B -> C -> D -> E -> DONE
        # -------------------------------------------------
        if self.phase == self.Phase.A_PARK and elapsed >= self.T_a:
            self.phase = self.Phase.B_NON_SINGULAR
            self.phase_start = t_now
            self._reset_integrators()
            self._just_switched_phase = True
            return

        if self.phase == self.Phase.B_NON_SINGULAR and elapsed >= self.T_b:
            # Cache pose using actual q
            self.x_b = self._fk_pos(q)
            self.x_c_start = self.x_b + self.linear_offset

            # Circle center so theta=0 starts at x_c_start
            self.circle_center = self.x_c_start - np.array([[-self.radius], [0.0], [0.0]])
            v0 = self.x_c_start - self.circle_center
            self.theta0 = np.arctan2(float(v0[1, 0]), float(v0[0, 0]))

            self.phase = self.Phase.C_LINEAR
            self.phase_start = t_now
            self._reset_integrators()
            self._just_switched_phase = True

            # New marker segment
            self._reset_marker_new_segment()
            self._set_marker_color_for_current_run()
            return

        if self.phase == self.Phase.C_LINEAR and elapsed >= self.T_c:
            self.phase = self.Phase.D_CROWN_W1
            self.phase_start = t_now
            self._reset_integrators()
            self._just_switched_phase = True
            return

        if self.phase == self.Phase.D_CROWN_W1 and elapsed >= self.T_d:
            self.phase = self.Phase.E_CROWN_W2
            self.phase_start = t_now
            self._reset_integrators()
            self._just_switched_phase = True
            return

        if self.phase == self.Phase.E_CROWN_W2 and elapsed >= self.T_d:
            # Task3 run completed
            self.phase = self.Phase.DONE
            self.phase_start = t_now
            self._reset_integrators()
            self._just_switched_phase = True
            return

        # -------------------------------------------------
        # Task3 DONE: OP_PD -> OP_PID transition
        # (JS experiment already completed before reaching here)
        # -------------------------------------------------
        if self.phase == self.Phase.DONE:
            if self.run_id == 0:
                rospy.loginfo("=" * 60)
                rospy.loginfo("[OP] OP_PD run completed. Starting OP_PID.")
                rospy.loginfo("=" * 60)
                self.run_id = 1
                self.mode = self.Mode.OP_PID

                self.phase = self.Phase.A_PARK
                self.phase_start = t_now
                self._reset_integrators()
                self._just_switched_phase = True

                self._reset_marker_new_segment()
                self._set_marker_color_for_current_run()
                return

            # run_id == 1 → All experiments completed
            # Stay in DONE
            return

        # -------------------------------------------------
        # Joint-Space Control Phases
        # JS_REGULATE -> JS_TRACK_SLOW -> JS_TRACK_FAST -> JS_DONE
        # After first run (JP_PD), restart with JP_PID
        # -------------------------------------------------
        if self.phase == self.Phase.JS_REGULATE and elapsed >= self.duration_js_regulate:
            rospy.loginfo(f"[JS-{self.mode.name}] Regulation complete. Starting slow tracking.")
            self.phase = self.Phase.JS_TRACK_SLOW
            self.phase_start = t_now
            self._reset_integrators()
            self._just_switched_phase = True
            return

        if self.phase == self.Phase.JS_TRACK_SLOW and elapsed >= self.duration_js_track_slow:
            rospy.loginfo(f"[JS-{self.mode.name}] Slow tracking complete. Starting fast tracking.")
            self.phase = self.Phase.JS_TRACK_FAST
            self.phase_start = t_now
            self._reset_integrators()
            self._just_switched_phase = True
            return

        if self.phase == self.Phase.JS_TRACK_FAST and elapsed >= self.duration_js_track_fast:
            rospy.loginfo(f"[JS-{self.mode.name}] Fast tracking complete.")
            self.phase = self.Phase.JS_DONE
            self.phase_start = t_now
            self._reset_integrators()
            self._just_switched_phase = True
            return

        if self.phase == self.Phase.JS_DONE:
            if self.js_run_id == 0:
                # First run (JP_PD) completed, start second run with JP_PID
                rospy.loginfo("=" * 60)
                rospy.loginfo("[JS] JP_PD run completed. Starting JP_PID.")
                rospy.loginfo("=" * 60)
                self.js_run_id = 1
                self.mode = self.Mode.JP_PID
                self.phase = self.Phase.JS_REGULATE
                self.phase_start = t_now
                self._reset_integrators()
                self._just_switched_phase = True
                return
            else:
                # JP_PID run completed - transition to Operational Space experiment
                rospy.loginfo("=" * 60)
                rospy.loginfo("[JS] Joint-Space experiment completed (JP_PD + JP_PID).")
                rospy.loginfo("[OP] Starting Operational-Space experiment with OP_PD.")
                rospy.loginfo("=" * 60)
                
                self.run_id = 0  # Reset for OP experiment
                self.mode = self.Mode.OP_PD
                self.phase = self.Phase.A_PARK
                self.phase_start = t_now
                self._reset_integrators()
                self._just_switched_phase = True
                
                self._reset_marker_new_segment()
                self._set_marker_color_for_current_run()
                return

    # =========================================================
    # Integrator handling
    # =========================================================
    def _reset_integrators(self):
        self.int_x[:] = 0.0
        self.int_q[:] = 0.0

    # =========================================================
    # Reference generation
    # =========================================================
    def _compute_qdot_r(self, t_now, q, dt):
        # Task3 a,b: joint motion
        if self.phase in [self.Phase.A_PARK, self.Phase.B_NON_SINGULAR]:
            qd = self.q_park if self.phase == self.Phase.A_PARK else self.q_safe
            return self._qdot_r_joint_to_target(q, qd)

        # -------------------------------------------------
        # Joint-Space Adaptive Control Phases (JS_REGULATE, JS_TRACK_SLOW, JS_TRACK_FAST)
        # -------------------------------------------------
        if self.phase in [self.Phase.JS_REGULATE, self.Phase.JS_TRACK_SLOW, 
                          self.Phase.JS_TRACK_FAST, self.Phase.JS_DONE]:
            return self._qdot_r_joint_space(t_now, q, dt)

        # Operational-space phases (Task3 c,d,e)
        xdot_r = self._xdot_r_operational(t_now, q, dt)
        J = self._jacobian_linear(q)
        qdot_r = np.linalg.pinv(J) @ xdot_r
        qdot_r = np.clip(qdot_r, -self.qdot_r_max, self.qdot_r_max)

        return qdot_r

    def _qdot_r_joint_space(self, t_now, q, dt):
        """
        Compute reference velocity for joint-space adaptive control.
        
        PD-like (JP_PD):  q̇_r = q̇_d - Kp * (q - q_d)
        PID-like (JP_PID): q̇_r = q̇_d - Kp * (q - q_d) - Ki * ∫(q - q_d)dt
        
        where (q_d, q̇_d, q̈_d) come from _joint_space_trajectory()
        """
        # Get desired joint trajectory
        q_d, qdot_d, qddot_d = self._joint_space_trajectory(t_now)
        
        # Position error: Δq = q - q_d
        dq = q - q_d
        
        if self.mode == self.Mode.JP_PD:
            # PD-like reference velocity: q̇_r = q̇_d - Kp * Δq
            qdot_r = qdot_d - (self.Kp_js @ dq)
            
        elif self.mode == self.Mode.JP_PID:
            # PID-like: add integral term
            # Update integral error with anti-windup
            self.int_q += dq * dt
            self.int_q = np.clip(self.int_q, -self.int_q_max, self.int_q_max)
            
            # q̇_r = q̇_d - Kp * Δq - Ki * ∫Δq dt
            qdot_r = qdot_d - (self.Kp_js @ dq) - (self.Ki_js @ self.int_q)
        else:
            # Fallback to simple proportional
            qdot_r = qdot_d - (self.Kp_js @ dq)
        
        # Use lower velocity limit for joint-space phases
        qdot_r = np.clip(qdot_r, -self.qdot_r_max_js, self.qdot_r_max_js)
        return qdot_r

    def _joint_space_trajectory(self, t_now):
        """
        Generate joint-space trajectory for regulation and tracking.
        
        Returns: (q_d, qdot_d, qddot_d) as (nq x 1) arrays
        """
        tau = (t_now - self.phase_start).to_sec()
        
        # Default: regulation at q_regulation
        q_d = self.q_regulation.copy()
        qdot_d = np.zeros((self.nq, 1))
        qddot_d = np.zeros((self.nq, 1))
        
        if self.phase == self.Phase.JS_REGULATE:
            # Regulation: constant desired position
            pass  # q_d = q_regulation, qdot_d = 0, qddot_d = 0
            
        elif self.phase == self.Phase.JS_TRACK_SLOW:
            # Slow sinusoidal tracking around regulation point
            w = self.w_js_slow
            A = self.q_amplitude
            
            sin_wt = np.sin(w * tau)
            cos_wt = np.cos(w * tau)
            
            q_d = self.q_regulation + A * np.array([[sin_wt], [sin_wt], [sin_wt]])
            qdot_d = A * w * np.array([[cos_wt], [cos_wt], [cos_wt]])
            qddot_d = -A * w**2 * np.array([[sin_wt], [sin_wt], [sin_wt]])
            
        elif self.phase == self.Phase.JS_TRACK_FAST:
            # Fast sinusoidal tracking around regulation point
            w = self.w_js_fast
            A = self.q_amplitude
            
            sin_wt = np.sin(w * tau)
            cos_wt = np.cos(w * tau)
            
            q_d = self.q_regulation + A * np.array([[sin_wt], [sin_wt], [sin_wt]])
            qdot_d = A * w * np.array([[cos_wt], [cos_wt], [cos_wt]])
            qddot_d = -A * w**2 * np.array([[sin_wt], [sin_wt], [sin_wt]])
            
        elif self.phase == self.Phase.JS_DONE:
            # Hold at regulation position
            pass
        
        return q_d, qdot_d, qddot_d

    def _qdot_r_joint_to_target(self, q, q_target):
        dq = q - q_target
        qdot_r = - (self.Kp_q @ dq)
        qdot_r = np.clip(qdot_r, -self.qdot_r_max, self.qdot_r_max)
        return qdot_r

    def _xdot_r_operational(self, t_now, q, dt):
        Xd, Xdot_d = self._cartesian_desired(t_now)
        X = self._fk_pos(q)

        # Document convention: ΔX = X - Xd
        dX = X - Xd

        # (4.5) Xdot_r = Xdot_d - Kp * ΔX
        Xdot_r = Xdot_d - (self.Kp_x @ dX)

        # (4.8) PID-like: - Ki ∫ΔX (only meaningful when feedback is active)
        if self.mode == self.Mode.OP_PID:
            self.int_x += dX * dt
            self.int_x = np.clip(self.int_x, -0.02, 0.02)  # tighter bound
            Xdot_r -= (0.3 * self.Ki_x @ self.int_x)

        # Velocity saturation
        Xdot_r = np.clip(Xdot_r, -self.xdot_r_max, self.xdot_r_max)

        return Xdot_r

    def _cartesian_desired(self, t_now):
        tau = (t_now - self.phase_start).to_sec()

        # Safety fallback if not initialized
        if self.x_b is None or self.x_c_start is None or self.circle_center is None:
            x_hold = self._fk_pos(self.q_safe)
            return x_hold, np.zeros((3, 1))

        # Task3 c) linear segment
        if self.phase == self.Phase.C_LINEAR:
            T = self.T_c
            s = np.clip(tau / T, 0.0, 1.0)
            Xd = (1.0 - s) * self.x_b + s * self.x_c_start
            Xdot_d = (self.x_c_start - self.x_b) / T if tau < T else np.zeros((3, 1))
            return Xd, Xdot_d

        # Task3 d,e) crown
        if self.phase in [self.Phase.D_CROWN_W1, self.Phase.E_CROWN_W2]:
            w = self.w1 if self.phase == self.Phase.D_CROWN_W1 else self.w2
            theta = self.theta0 + w * tau
            c = self.circle_center
            k = 10.0

            Xd = np.array([
                [c[0, 0] + self.radius * np.cos(theta)],
                [c[1, 0] + self.radius * np.sin(theta)],
                [c[2, 0] + self.z_amp * np.sin(k * theta)],
            ])

            Xdot_d = np.array([
                [-self.radius * w * np.sin(theta)],
                [ self.radius * w * np.cos(theta)],
                [ self.z_amp * k * w * np.cos(k * theta)],
            ])
            return Xd, Xdot_d

        # Default: hold last known start
        return self.x_c_start.copy(), np.zeros((3, 1))

    # =========================================================
    # Model interfaces
    # =========================================================
    def _fk_pos(self, q):
        T = np.array(self.model.T_ee_w_fn(*q.flatten()), dtype=float)
        return T[0:3, 3].reshape(3, 1)

    def _jacobian_linear(self, q):
        # Model provides J_ee_w_fn which returns 6xn Jacobian
        # Take first 3 rows for linear (position) Jacobian
        J_full = np.array(self.model.J_ee_w_fn(*q.flatten()), dtype=float)
        return J_full[0:3, :]

    # =========================================================
    # Utility
    # =========================================================
    def _current_joint_target(self):
        if self.phase == self.Phase.A_PARK:
            return self.q_park
        if self.phase == self.Phase.B_NON_SINGULAR:
            return self.q_safe
        if self.phase in [self.Phase.JS_REGULATE, self.Phase.JS_TRACK_SLOW, 
                          self.Phase.JS_TRACK_FAST, self.Phase.JS_DONE]:
            return self.q_regulation
        # Default: for non-joint phases, return something stable
        return self.q_safe

