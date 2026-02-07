#include <tum_ics_ur10_controller_tutorial/operational_space_controller_6dof.h>
#include <tum_ics_ur_robot_msgs/ControlData.h>
#include <Eigen/SVD>
#include <Eigen/Geometry>

namespace tum_ics_ur_robot_lli
{
  namespace RobotControllers
  {

    OperationalSpaceController6DOF::OperationalSpaceController6DOF(double weight, const QString &name)
        : ControlEffort(name, SPLINE_TYPE, JOINT_SPACE, weight),
          is_first_iter_(true),
          current_state_(State::INIT),
          controller_type_(ControllerType::PD),
          control_space_(ControlSpace::JOINT),
          model_("ur10_model")
    {
      // UR10 DH parameters (meters)
      d1_ = 0.1273;   // Link offset d1
      a2_ = -0.612;   // Link length a2
      a3_ = -0.5723;  // Link length a3
      d4_ = 0.163941; // Link offset d4
      d5_ = 0.1157;   // Link offset d5
      d6_ = 0.0922;   // Link offset d6
      // Initialize gain matrices
      Kp_PD_ = 20.0 * Matrix3d::Identity();
      Kd_PD_ = 20.0 * Matrix3d::Identity();
      Kreg_PD_ = Matrix6d::Zero();
      Kreg_PD_.block<3, 3>(0, 0) = 20.0 * Matrix3d::Identity();
      Kreg_PD_.block<3, 3>(3, 3) = 1.0 * Matrix3d::Identity();

      Kp_PID_ = 5.0 * Matrix3d::Identity();
      Kd_PID_ = 5.0 * Matrix3d::Identity();
      Ki_PID_ = 5.0 * Matrix3d::Identity();

      // Initialize time parameters
      t_INIT_ = 5.0;
      t_LINEAR_ = 10.0;
      // increasing the time of tracking phase leads to the robots velocity to decrease -> hence velocity limits arent violated 
      t_TRACK_SLOW_ = 600.0;
      t_TRACK_FAST_ = 30.0;

      // Initialize poses
      start_track_pose_ << 0.0, -M_PI / 3.0, -M_PI + M_PI / 2.0, 0.0, 0.0, 0.0;
      non_singular_pose_ << 0.0, 0.0, -M_PI / 2.0, 0.0, 0.0, 0.0;
      
      // Initialize circular trajectory parameters (like Python)
      orbital_center_.setZero();
      circle_radius_ = 0.20;      // Same as Python's circle_radius_nominal
      vertical_amplitude_ = 0.02; // Same as Python's vertical_oscillation_amplitude

      // Initialize adaptive parameters
      theta_.setZero();
      ro_.setConstant(0.1);

      // Initialize error accumulators
      integral_error_acc_.setZero();

      // Initialize state variables
      q_.setZero();
      X_.setZero();
      last_q_.setZero();
      last_X_.setZero();

      // ROS Publishers
      joint_states_des_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/joint_states_des", 10);
      lyapunov_pub_ = nh_.advertise<std_msgs::Float32>("/lyapunov", 10);
      steady_state_error_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/steady_state_error", 10);
      cartesian_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/cart_coor", 10);
      control_data_pub_ = nh_.advertise<tum_ics_ur_robot_msgs::ControlData>("operational_space_controller_data", 1);
      
    }

    OperationalSpaceController6DOF::~OperationalSpaceController6DOF()
    {
    }

    // Setters for initial, home, and park configurations (same as for simple effort controller)
    void OperationalSpaceController6DOF::setQInit(const JointState &q_init)
    {
      q_init_ = q_init;
    }
    void OperationalSpaceController6DOF::setQHome(const JointState &q_home)
    {
      q_home_ = q_home;
    }
    void OperationalSpaceController6DOF::setQPark(const JointState &q_park)
    {
      q_park_ = q_park;
    }
    
    bool OperationalSpaceController6DOF::init()
    {
      ROS_WARN_STREAM("OperationalSpaceController6DOF::init");

      // CRITICAL: Initialize the ur_model - must be called before using any model functions!
      if (!model_.initRequest(nh_))
      {
        ROS_ERROR_STREAM("Error: initializing ur_model failed!");
        return false;
      }
      ROS_INFO_STREAM("ur_model initialized successfully");

      std::string ns = "~operational_space_ctrl";

      // Load gains from parameter server (optional - uses defaults if not present)
      if (ros::param::has(ns))
      {
        std::vector<double> vec;

        // P gains for Cartesian space
        if (ros::param::get(ns + "/gains_p_cart", vec) && vec.size() >= 3)
        {
          for (int i = 0; i < 3; i++)
            Kp_PD_(i, i) = vec[i];
        }

        // D gains for Cartesian space
        if (ros::param::get(ns + "/gains_d_cart", vec) && vec.size() >= 3)
        {
          for (int i = 0; i < 3; i++)
            Kd_PD_(i, i) = vec[i];
        }

        // Regularization gains
        if (ros::param::get(ns + "/gains_reg", vec) && vec.size() >= 6)
        {
          for (int i = 0; i < 6; i++)
            Kreg_PD_(i, i) = vec[i];
        }

        // Time parameters
        ros::param::get(ns + "/t_init", t_INIT_);
        ros::param::get(ns + "/t_linear", t_LINEAR_);
        ros::param::get(ns + "/t_track_slow", t_TRACK_SLOW_);
        ros::param::get(ns + "/t_track_fast", t_TRACK_FAST_);
      }

      ROS_WARN_STREAM("Kp_PD: \n" << Kp_PD_);
      ROS_WARN_STREAM("Kd_PD: \n" << Kd_PD_);
      ROS_WARN_STREAM("Kreg_PD: \n" << Kreg_PD_);
      ROS_WARN_STREAM("Time params - INIT: " << t_INIT_ << ", LINEAR: " << t_LINEAR_ 
                      << ", TRACK_SLOW: " << t_TRACK_SLOW_ << ", TRACK_FAST: " << t_TRACK_FAST_);



      return true;
    }
    bool OperationalSpaceController6DOF::start()
    {
      ROS_WARN_STREAM("OperationalSpaceController6DOF::start");
      return true;
    }
    Vector6d OperationalSpaceController6DOF::update(const RobotTime &time, const JointState &state)
    {
      // First iteration initialization
      if (is_first_iter_)
      {
        q_start_ = state.q;
        t0_ = time.tD();
        last_t_ = t0_;
        last_q_ = state.q;

        // Get initial Cartesian position using our FK
        X_ = getEndEffectorPosition(state.q);
        last_X_ = X_;

        ROS_WARN_STREAM("START [RAD]: \n" << q_start_.transpose());
        ROS_WARN_STREAM("Initial EE position X_: " << X_.transpose() << " (should be ~0.5-1.0m values)");
        
        // Also verify track position FK
        Vector3d track_pos = getEndEffectorPosition(start_track_pose_);
        ROS_WARN_STREAM("Track pose FK: " << track_pos.transpose());
        
        is_first_iter_ = false;
      }

      // Update current state
      q_ = state.q;
      X_ = getEndEffectorPosition(state.q);

      // Calculate elapsed time
      double t_elapsed = time.tD() - t0_;
      double dt = 0.002; // Assume 500Hz control loop (adjust as needed)

      // Update state machine
      updateStateMachine(t_elapsed);

      // Check for singularity (manipulability)
      if (control_space_ == ControlSpace::CARTESIAN)
      {
        Eigen::Matrix<double, 6, 6> J = computeJacobian(state.q);
        double w = computeManipulability(J);
        if (w < 0.0)
        {
          ROS_WARN_STREAM("Near singularity! Switching to joint space. w=" << w);
          control_space_ = ControlSpace::JOINT;
          current_state_ = State::END;
        }
      }

      // Generate desired trajectory
      Vector6d q_des, qdot_des, qddot_des;
      Vector3d x_des, xdot_des, xddot_des;
      //ROS_WARN_STREAM("x_des: " << x_des.transpose() << " xdot_des: " << xdot_des.transpose() << " xddot_des: " << xddot_des.transpose());
      generateTrajectory(time.tD(), q_des, qdot_des, qddot_des, x_des, xdot_des, xddot_des);

      // Compute control torque
      Vector6d tau;
      tau.setZero();

      if (control_space_ == ControlSpace::JOINT)
      {
        tau = jointSpaceController(state, q_des, qdot_des, qddot_des, dt);
      }
      else // CARTESIAN
      {
        Vector3d xdot_current = getEndEffectorVelocity(state.q, state.qp);
        
        // DEBUG: Log x_des to verify it's not zero
        ROS_INFO_STREAM_THROTTLE(0.5, "[CART] x_des=" << x_des.transpose() 
                                 << " X_=" << X_.transpose()
                                 << " delta=" << (X_ - x_des).transpose());
        
        tau = cartesianSpaceController(state, X_, xdot_current, x_des, xdot_des, xddot_des, dt);
      }

      // Publish debug data
      tum_ics_ur_robot_msgs::ControlData msg;
      msg.header.stamp = ros::Time::now();
      msg.time = time.tD();
      for (int i = 0; i < STD_DOF; i++)
      {
        msg.q[i] = state.q(i);
        msg.qp[i] = state.qp(i);
        msg.qpp[i] = state.qpp(i);
        msg.qd[i] = q_des(i);
        msg.qpd[i] = qdot_des(i);
        msg.Dq[i] = state.q(i) - q_des(i);
        msg.Dqp[i] = state.qp(i) - qdot_des(i);
        msg.torques[i] = tau(i);
      }
      control_data_pub_.publish(msg);

      return tau;
    }
    bool OperationalSpaceController6DOF::stop()
    {
      ROS_WARN_STREAM("OperationalSpaceController6DOF::stop");
      return true;
    }
    void OperationalSpaceController6DOF::updateStateMachine(double t_elapsed)
    {
      State new_state = current_state_;

      if (t_elapsed > t_INIT_ + t_LINEAR_ + t_TRACK_SLOW_ + t_TRACK_FAST_ && 
          current_state_ == State::TRACK_FAST)
      {
        new_state = State::END;
        control_space_ = ControlSpace::JOINT;
        //last_t_ = ros::Time::now().toSec();
        last_q_ = q_;
        last_X_ = X_;
      }
      else if (t_elapsed > t_INIT_ + t_LINEAR_ + t_TRACK_SLOW_ && 
               current_state_ == State::TRACK_SLOW)
      {
        new_state = State::TRACK_FAST;
        control_space_ = ControlSpace::CARTESIAN;
        //last_t_ = ros::Time::now().toSec();
        last_q_ = q_;
        last_X_ = X_;
      }
      else if (t_elapsed > t_INIT_ + t_LINEAR_ && 
               current_state_ == State::LINEAR)
      {
        new_state = State::TRACK_SLOW;
        control_space_ = ControlSpace::CARTESIAN;
        //last_t_ = ros::Time::now().toSec();
        last_q_ = q_;
        last_X_ = X_;
      }
      else if (t_elapsed > t_INIT_ && current_state_ == State::INIT)
      {
        new_state = State::LINEAR;
        control_space_ = ControlSpace::CARTESIAN;
        //last_t_ = t_elapsed + t0_;
        //last_t_ = ros::Time::now().toSec();
        last_q_ = q_;
        last_X_ = X_;
      }

      // If state changed, update transition variables
      if (new_state != current_state_)
      {
        ROS_INFO_STREAM("State transition: " << static_cast<int>(current_state_) 
                        << " -> " << static_cast<int>(new_state));
        
        // Compute orbital_center_ when transitioning to TRACK_SLOW (like Python)
        // Python: self.orbital_center = self.linear_start_pose - shift
        if (new_state == State::TRACK_SLOW)
        {
          // orbital_center_ is offset from the current Cartesian position
          // In Python: c = self.linear_start_pose (which is where LINEAR ends)
          // The circle is centered such that the initial point is on the circle
          orbital_center_ = X_;  // Start circle from current position
          orbital_center_(0) -= circle_radius_;  // Offset so we start at angle=0 on the circle
          ROS_INFO_STREAM("Computed orbital_center_: " << orbital_center_.transpose());
        }
        
        current_state_ = new_state;
        // FIXED: Use consistent time source! t_elapsed is based on time.tD(), not ros::Time::now()
        // We need to store the controller time at transition, not wall clock time
        last_t_ = t_elapsed + t0_;  // Convert back to absolute controller time
        // last_q_ = q_;
        // last_X_ = X_;
      }
    }
    void OperationalSpaceController6DOF::generateTrajectory(double t,
                                                            Vector6d &q_des, Vector6d &qdot_des, Vector6d &qddot_des,
                                                            Vector3d &x_des, Vector3d &xdot_des, Vector3d &xddot_des)
    {
      q_des.setZero();
      qdot_des.setZero();
      qddot_des.setZero();
      x_des.setZero();
      xdot_des.setZero();
      xddot_des.setZero();

      // Get track position in Cartesian space (compute FK for start_track_pose_)
      Vector3d track_pos_cartesian = getEndEffectorPosition(start_track_pose_);

      double omega = 2.0 * M_PI / t_TRACK_SLOW_;
      double omega_z = omega * 15.0;
      double omega_fast = 2.0 * M_PI / t_TRACK_FAST_;
      double omega_z_fast = omega_fast * 15.0;

      Vector3d amplitude;
      amplitude << track_pos_cartesian(0), track_pos_cartesian(0), 0.05;


      // FIXED: offset should be time since current state started, not absolute time
      // Python uses: tau = (t_now - self.phase_entry_time).to_sec()
      double offset = t - last_t_;
      
      // DEBUG: Log trajectory generation details
      ROS_INFO_STREAM_THROTTLE(0.5, "[TRAJ] State=" << static_cast<int>(current_state_) 
                               << " t=" << t << " last_t_=" << last_t_ << " offset=" << offset
                               << " last_X_=" << last_X_.transpose()
                               << " track_pos=" << track_pos_cartesian.transpose());

      switch (current_state_)
      {
      case State::INIT:
        // Linear interpolation to non-singular pose in joint space
        q_des = (non_singular_pose_ / t_INIT_) * offset;
        break;

      case State::LINEAR:
        // Linear interpolation in Cartesian space to tracking start position
        x_des = last_X_ + ((track_pos_cartesian - last_X_) / t_LINEAR_) * offset;
        break;

      case State::TRACK_SLOW:
      {
        // FIXED: Circle centered at orbital_center_, not at origin (like Python)
        // Python: x = c[0] + radius*cos(theta), y = c[1] + radius*sin(theta), z = c[2] + amp*sin(k*theta)
        double k = 10.0;  // z oscillation frequency multiplier (like Python)
        double theta = omega * offset;
        
        /*
        //orbital centered trajectory generation 
        x_des(0) = orbital_center_(0) + circle_radius_ * std::cos(theta);
        x_des(1) = orbital_center_(1) + circle_radius_ * std::sin(theta);
        x_des(2) = orbital_center_(2) + vertical_amplitude_ * std::sin(k * theta);

        xdot_des(0) = -circle_radius_ * omega * std::sin(theta);
        xdot_des(1) = circle_radius_ * omega * std::cos(theta);
        xdot_des(2) = vertical_amplitude_ * k * omega * std::cos(k * theta);
        */

        /*
        Python code:
        x_des = np.array([np.cos(omega * (t - self.last_t)), np.sin(omega * (t - self.last_t)), np.cos(omega_z * (t - self.last_t))]) * amplitude
        offset = t - self.last_t 
        xdot_des = amplitude * np.array([
            -np.sin(omega   * offset) * omega,
            np.cos(omega   * offset) * omega,
            -np.sin(omega_z * offset) * omega_z,
        ])
        */
        //cpp code:
        x_des(0) = amplitude(0) * std::cos(omega * offset);
        x_des(1) = amplitude(1) * std::sin(omega * offset);
        x_des(2) = amplitude(2) * std::cos(omega_z * offset);

        xdot_des(0) = -amplitude(0) * std::sin(omega * offset) * omega;
        xdot_des(1) = amplitude(1) * std::cos(omega * offset) * omega;
        xdot_des(2) = -amplitude(2) * std::sin(omega_z * offset) * omega_z;
        break;
      }

      case State::TRACK_FAST:
      {
        // FIXED: Same fix for fast tracking
        double k = 10.0;
        double theta = omega_fast * offset;
        
        x_des(0) = orbital_center_(0) + circle_radius_ * std::cos(theta);
        x_des(1) = orbital_center_(1) + circle_radius_ * std::sin(theta);
        x_des(2) = orbital_center_(2) + vertical_amplitude_ * std::sin(k * theta);

        xdot_des(0) = -circle_radius_ * omega_fast * std::sin(theta);
        xdot_des(1) = circle_radius_ * omega_fast * std::cos(theta);
        xdot_des(2) = vertical_amplitude_ * k * omega_fast * std::cos(k * theta);

        x_des(0) = amplitude(0) * std::cos(omega_fast * offset);
        x_des(1) = amplitude(1) * std::sin(omega_fast * offset);
        x_des(2) = amplitude(2) * std::cos(omega_z_fast * offset);

        xdot_des(0) = -amplitude(0) * std::sin(omega_fast * offset) * omega_fast;
        xdot_des(1) = amplitude(1) * std::cos(omega_fast * offset) * omega_fast;
        xdot_des(2) = -amplitude(2) * std::sin(omega_z_fast * offset) * omega_z_fast;
        break;
      }

      case State::END:
      default:
        q_des = closestEquivalentAngles(non_singular_pose_, q_);
        break;
      }
    }
    Vector6d OperationalSpaceController6DOF::jointSpaceController(const JointState &state,
                                                                   const Vector6d &q_des,
                                                                   const Vector6d &qdot_des,
                                                                   const Vector6d &qddot_des,
                                                                   double dt)
    {
      Vector6d tau;
      tau.setZero();

      // Errors
      Vector6d delta_q = state.q - q_des;
      Vector6d delta_qdot = state.qp - qdot_des;

      // Reference velocity with proportional term
      Vector6d qrdot = qdot_des - Kreg_PD_ * delta_q / 20.0; // Normalize by Kreg scale

      // Sliding variable
      Vector6d S_q = state.qp - qrdot;

      // Simple PD control in joint space (can be extended with regressor-based adaptive control)
      // tau = -Kd * S_q approximates the adaptive control law
      tau = -Kreg_PD_ * S_q;

      return tau;
    }
    Vector6d OperationalSpaceController6DOF::cartesianSpaceController(const JointState &state,
                                                                       const Vector3d &x_current,
                                                                       const Vector3d &xdot_current,
                                                                       const Vector3d &x_des,
                                                                       const Vector3d &xdot_des,
                                                                       const Vector3d &xddot_des,
                                                                       double dt)
    {
      Vector6d tau;
      tau.setZero();

      Vector6d Yr_theta; // Placeholder for regressor-based adaptive control term

      // Cartesian errors
      Vector3d delta_x = x_current - x_des;
      // ROS_WARN_STREAM("delta_x: " << delta_x.transpose());  
      Vector3d delta_xdot = xdot_current - xdot_des;

      // Update integral error
      integral_error_acc_ += delta_x * dt;

      // Get full Jacobian and extract translational part (rows 0-2)
      Eigen::Matrix<double, 6, 6> J_full = computeJacobian(state.q);
      Eigen::Matrix<double, 3, 6> J_ee = J_full.block<3, 6>(0, 0);  // Translational Jacobian

      // Compute pseudoinverse of Jacobian
      Eigen::MatrixXd J_pinv = pseudoInverse(J_ee);

      Eigen::Matrix<double,6,6> Jdot_ee_w  = computeJacobianDotTranslational(state.q, state.qp);
      Eigen::Matrix<double, 3, 6> Jdot_ee_w_translation = Jdot_ee_w.block<3, 6>(0, 0); // Get translational part of Jacobian derivative in world frame
      //Eigen::Matrix<double, 3, 6> Jdot_e_w_linear = Jdot_ee_w.block<3, 6>(3, 0); // Extract translational part of Jacobian derivative
      //PD controller seems to be correctly
      if (controller_type_ == ControllerType::PD)
      {
        // Reference Cartesian velocity
        Vector3d xrdot = xdot_des - Kp_PD_ * delta_x;
        Vector3d xrddot = xddot_des - Kp_PD_ * delta_xdot;
        // Map to joint space
        Vector6d qrdot = J_pinv * xrdot;
        Vector6d qrddot = J_pinv * (xrddot- Jdot_ee_w_translation * qrdot); // Jdot_ee is the time derivative of the Jacobian 

        // Sliding variable in joint space
        Vector6d S_q = state.qp - qrdot;

        //TODO: implement missing parts aka ro, computeRegressor and
        //Vector6d Yr = computeRegressor(state.q, state.qp, qrdot, xddot_des); // Compute regressor based on current state and desired acceleration
        //Vector6d theta -= (ro * (Yr.transpose() * S_q)) * dt; // Update adaptive parameters using gradient descent
        Yr_theta = computeYr_theta(state.q, state.qp, qrdot, qrddot); 
        // Simple PD control (can be extended with adaptive control)
        tau = -Kreg_PD_ * S_q + Yr_theta;


        
       
      }
      else if (controller_type_ == ControllerType::PID)
      {
        
        //TODO: implement correct version of PID Controller 
        // PID in Cartesian space
        Vector3d xrdot = xdot_des - Kp_PID_ * delta_x - Ki_PID_ * integral_error_acc_;
        Vector3d xrddot = xddot_des - Kp_PID_ * delta_xdot - Ki_PID_ * delta_x;
        Vector6d qrdot = J_pinv * xrdot;
        // TODO: implement Jdot_ee helper function that calculates the time derivative of the Jacobian based on current q and qp
        Vector6d qrddot = J_pinv * (xrddot- Jdot_ee_w_translation * qrdot); // Jdot_ee is the time derivative of the Jacobian 
        Vector6d S_q = state.qp - qrdot;
        //TODO: add helper function that calculate M, C, G and maybe also regressor Y from model.zip files
        //Yr_theta = M * qrddot + C * qrdot + G; // Regressor-based adaptive control law (simplified)
        Yr_theta = computeYr_theta(state.q, state.qp, qrdot, qrddot); 
        tau = -Kd_PID_(0, 0) * S_q + Yr_theta;  // Use scalar Kd for simplicity       
      }

      // Publish Cartesian tracking data
      std_msgs::Float32MultiArray delta_msg;
      delta_msg.data = {static_cast<float>(delta_x(0)), 
                        static_cast<float>(delta_x(1)), 
                        static_cast<float>(delta_x(2))};
      steady_state_error_pub_.publish(delta_msg);

      std_msgs::Float32MultiArray x_des_msg;
      x_des_msg.data = {static_cast<float>(x_des(0)), 
                        static_cast<float>(x_des(1)), 
                        static_cast<float>(x_des(2))};
      joint_states_des_pub_.publish(x_des_msg);

      std_msgs::Float32MultiArray x_msg;
      x_msg.data = {static_cast<float>(x_current(0)), 
                    static_cast<float>(x_current(1)), 
                    static_cast<float>(x_current(2))};
      cartesian_pub_.publish(x_msg);

      return tau;
    }
    Vector6d OperationalSpaceController6DOF::closestEquivalentAngles(const Vector6d &q_ref, 
                                                                      const Vector6d &q_current)
    {
      Vector6d result;
      for (int i = 0; i < 6; i++)
      {
        result(i) = q_ref(i) + 2.0 * M_PI * std::round((q_current(i) - q_ref(i)) / (2.0 * M_PI));
      }
      return result;
    }
    Eigen::MatrixXd OperationalSpaceController6DOF::pseudoInverse(const Eigen::MatrixXd &matrix, 
                                                                   double tolerance)
    {
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
      
      const auto &singularValues = svd.singularValues();
      Eigen::MatrixXd singularValuesInv(matrix.cols(), matrix.rows());
      singularValuesInv.setZero();

      for (int i = 0; i < singularValues.size(); ++i)
      {
        if (singularValues(i) > tolerance)
        {
          singularValuesInv(i, i) = 1.0 / singularValues(i);
        }
        else
        {
          singularValuesInv(i, i) = 0.0;
        }
      }

      return svd.matrixV() * singularValuesInv * svd.matrixU().transpose();
    }
    double OperationalSpaceController6DOF::computeManipulability(const Eigen::Matrix<double, 6, 6> &J)
    {
      // Yoshikawa manipulability measure: w = sqrt(det(J * J^T))
      Eigen::MatrixXd JJT = J * J.transpose();
      return std::sqrt(std::abs(JJT.determinant()));
    }

    // =========================================================================
    // Self-contained UR10 Kinematics Functions
    // =========================================================================
    
    Eigen::Matrix<double, 6, 6> OperationalSpaceController6DOF::computeJacobian(const Vector6d &q)
    {
      // Get Jacobian directly from ur_model (no numerical differentiation needed)
      // Transform from 0-frame to world frame using T_0_w rotation
      Eigen::Matrix4d T_0_w;
      T_0_w << -1, 0, 0, 0,
              0, -1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
      Eigen::Matrix3d R_0_w = T_0_w.block<3, 3>(0, 0);
      Eigen::Matrix<double, 6,6> Rotation_0_w;
      Rotation_0_w.setZero();
      Rotation_0_w.block<3, 3>(0, 0) = R_0_w;
      Rotation_0_w.block<3, 3>(3, 3) = R_0_w;
      
      // FIXED: Use q directly, not the uninitialized class member q_plus
      Eigen::Matrix<double,6,6> J_ef_0 = model_.J_ef_0(q);
      Eigen::Matrix<double,6,6> J_ef_w = Rotation_0_w * J_ef_0; // Transform to world frame
      return J_ef_w;
    }
    Eigen::Matrix<double, 6, 6> OperationalSpaceController6DOF::computeJacobianDotTranslational(const Vector6d &q, const Vector6d &qp)
    {
      Eigen::Matrix<double, 6, 6> Jp_ef_0 = model_.Jp_ef_0(q, qp); 
      //copied T_0_w from ur_model aka python version
      Eigen::Matrix4d T_0_w;
      T_0_w << -1, 0, 0, 0,
              0, -1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
      Eigen::Matrix3d R_0_w = T_0_w.block<3, 3>(0, 0);
      Eigen::Matrix<double, 6,6> Rotation_0_w;
      Rotation_0_w.setZero();
      Rotation_0_w.block<3, 3>(0, 0) = R_0_w;
      Rotation_0_w.block<3, 3>(3, 3) = R_0_w;
      Eigen::Matrix<double, 6, 6> Jp_ef_w = Rotation_0_w * Jp_ef_0; // Transform to base frame
      return Jp_ef_w;
    }

    Vector3d OperationalSpaceController6DOF::getEndEffectorPosition(const Vector6d &q)
    {
      //copied T_0_w from ur_model aka python version
      Eigen::Matrix4d T_0_w;
      T_0_w << -1, 0, 0, 0,
              0, -1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
      //get T_ef_0 from model
      //Eigen::Matrix4d T_ef_0 = model_.T_ef_0(q);
      Eigen::Matrix4d T_ef_0 = model_.T_ef_0(q).matrix(); 

      //Transform to world frame
      Eigen::Matrix4d T_ef_w = T_0_w * T_ef_0; 
      return T_ef_w.block<3,1>(0,3);
    }
    Vector3d OperationalSpaceController6DOF::getEndEffectorVelocity(const Vector6d &q, const Vector6d &qp)
    {

      /*
      def get_ee_w_dot(self):
        J_ee_w = self.model.J_ee_w_fn(*self.q)
        
        # Calculate the end-effector Twist in the world frame V_ee_w
        V_ee_w = J_ee_w @ self.qdot
        return V_ee_w[3:]
      */
      Eigen::Matrix<double, 6, 6> J = computeJacobian(q);
      Eigen:Matrix<double, 6, 1> V_ee_w = J * qp; // End-effector twist in world frame
      return V_ee_w.block<3, 1>(3, 0); //

      /*
      // xdot = J * qdot (linear velocity part only)
      Eigen::Matrix<double, 6, 6> J = computeJacobian(q);
      Eigen::Matrix<double, 3, 6> J_linear = J.block<3, 6>(0, 0);
      return J_linear * qp;      
      */

    }

    Vector6d OperationalSpaceController6DOF::computeYr_theta(const Vector6d& q, const Vector6d& qdot, const Vector6d& qrdot, const Vector6d& qrddot)
    {
      //get Regressor Y and theta_ from ur_model
      // Initalize the robot regressor matrix
      ur::URModel::Regressor Y = model_.regressor(q, qdot, qrdot, qrddot);
      
      // Initalize the robots parameter vector
      ur::URModel::Parameters th = model_.parameterInitalGuess(); 
      Vector6d Yr_theta = Y * th; // Compute the regressor-based control term
      return Yr_theta;
    }

  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli
