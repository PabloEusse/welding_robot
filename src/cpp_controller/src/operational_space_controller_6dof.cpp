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
          control_space_(ControlSpace::JOINT)
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
      t_INIT_ = 10.0;
      t_LINEAR_ = 10.0;
      t_TRACK_SLOW_ = 40.0;
      t_TRACK_FAST_ = 40.0;

      // Initialize poses
      start_track_pose_ << 0.0, -M_PI / 3.0, -M_PI + M_PI / 2.0, 0.0, 0.0, 0.0;
      non_singular_pose_ << 0.0, 0.0, -M_PI / 2.0, 0.0, 0.0, 0.0;

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
        if (w < 0.1)
        {
          ROS_WARN_STREAM("Near singularity! Switching to joint space. w=" << w);
          control_space_ = ControlSpace::JOINT;
          current_state_ = State::END;
        }
      }

      // Generate desired trajectory
      Vector6d q_des, qdot_des, qddot_des;
      Vector3d x_des, xdot_des, xddot_des;
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
      }
      else if (t_elapsed > t_INIT_ + t_LINEAR_ + t_TRACK_SLOW_ && 
               current_state_ == State::TRACK_SLOW)
      {
        new_state = State::TRACK_FAST;
        control_space_ = ControlSpace::CARTESIAN;
      }
      else if (t_elapsed > t_INIT_ + t_LINEAR_ && 
               current_state_ == State::LINEAR)
      {
        new_state = State::TRACK_SLOW;
        control_space_ = ControlSpace::CARTESIAN;
      }
      else if (t_elapsed > t_INIT_ && current_state_ == State::INIT)
      {
        new_state = State::LINEAR;
        control_space_ = ControlSpace::CARTESIAN;
      }

      // If state changed, update transition variables
      if (new_state != current_state_)
      {
        ROS_INFO_STREAM("State transition: " << static_cast<int>(current_state_) 
                        << " -> " << static_cast<int>(new_state));
        current_state_ = new_state;
        last_t_ = ros::Time::now().toSec();
        last_q_ = q_;
        last_X_ = X_;
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

      double offset = t - last_t_;

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
        x_des(0) = std::cos(omega * offset) * amplitude(0);
        x_des(1) = std::sin(omega * offset) * amplitude(1);
        x_des(2) = std::cos(omega_z * offset) * amplitude(2);

        xdot_des(0) = -std::sin(omega * offset) * omega * amplitude(0);
        xdot_des(1) = std::cos(omega * offset) * omega * amplitude(1);
        xdot_des(2) = -std::sin(omega_z * offset) * omega_z * amplitude(2);
        break;

      case State::TRACK_FAST:
        x_des(0) = std::cos(omega_fast * offset) * amplitude(0);
        x_des(1) = std::sin(omega_fast * offset) * amplitude(1);
        x_des(2) = std::cos(omega_z_fast * offset) * amplitude(2);

        xdot_des(0) = -std::sin(omega_fast * offset) * omega_fast * amplitude(0);
        xdot_des(1) = std::cos(omega_fast * offset) * omega_fast * amplitude(1);
        xdot_des(2) = -std::sin(omega_z_fast * offset) * omega_z_fast * amplitude(2);
        break;

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

      // Cartesian errors
      Vector3d delta_x = x_current - x_des;
      Vector3d delta_xdot = xdot_current - xdot_des;

      // Update integral error
      integral_error_acc_ += delta_x * dt;

      // Get full Jacobian and extract translational part (rows 0-2)
      Eigen::Matrix<double, 6, 6> J_full = computeJacobian(state.q);
      Eigen::Matrix<double, 3, 6> J_ee = J_full.block<3, 6>(0, 0);  // Translational Jacobian

      // Compute pseudoinverse of Jacobian
      Eigen::MatrixXd J_pinv = pseudoInverse(J_ee);

      if (controller_type_ == ControllerType::PD)
      {
        // Reference Cartesian velocity
        Vector3d xrdot = xdot_des - Kp_PD_ * delta_x;

        // Map to joint space
        Vector6d qrdot = J_pinv * xrdot;

        // Sliding variable in joint space
        Vector6d S_q = state.qp - qrdot;

        // Simple PD control (can be extended with adaptive control)
        tau = -Kreg_PD_ * S_q;
      }
      else if (controller_type_ == ControllerType::PID)
      {
        // PID in Cartesian space
        Vector3d xrdot = xdot_des - Kp_PID_ * delta_x - Ki_PID_ * integral_error_acc_;
        Vector6d qrdot = J_pinv * xrdot;
        Vector6d S_q = state.qp - qrdot;
        
        tau = -Kd_PID_(0, 0) * S_q;  // Use scalar Kd for simplicity
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

    // all of this can also be found in model.zip -> hence correct forward kinematics and jacobians
    Eigen::Affine3d OperationalSpaceController6DOF::forwardKinematics(const Vector6d &q)
    {
      // UR10 Forward Kinematics using DH convention
      // DH Parameters for UR10:
      // Joint | a      | d      | alpha   | theta
      // 1     | 0      | d1     | pi/2    | q1
      // 2     | a2     | 0      | 0       | q2
      // 3     | a3     | 0      | 0       | q3
      // 4     | 0      | d4     | pi/2    | q4
      // 5     | 0      | d5     | -pi/2   | q5
      // 6     | 0      | d6     | 0       | q6

      double c1 = std::cos(q(0)), s1 = std::sin(q(0));
      double c2 = std::cos(q(1)), s2 = std::sin(q(1));
      double c3 = std::cos(q(2)), s3 = std::sin(q(2));
      double c4 = std::cos(q(3)), s4 = std::sin(q(3));
      double c5 = std::cos(q(4)), s5 = std::sin(q(4));
      double c6 = std::cos(q(5)), s6 = std::sin(q(5));

      double c23 = std::cos(q(1) + q(2));
      double s23 = std::sin(q(1) + q(2));
      double c234 = std::cos(q(1) + q(2) + q(3));
      double s234 = std::sin(q(1) + q(2) + q(3));

      Eigen::Matrix4d T;
      T.setIdentity();

      // Position of end-effector
      double px = d6 * (c1 * c234 * c5 + s1 * s5) + c1 * (a3_ * c23 + a2_ * c2) - d5 * s1 - d4 * c1 * s234;
      double py = d6 * (s1 * c234 * c5 - c1 * s5) + s1 * (a3_ * c23 + a2_ * c2) + d5 * c1 - d4 * s1 * s234;
      double pz = d1_ + a3_ * s23 + a2_ * s2 - d6 * s234 * c5 - d4 * c234;

      T(0, 3) = px;
      T(1, 3) = py;
      T(2, 3) = pz;

      // Rotation matrix (simplified - can be expanded if needed)
      T(0, 0) = c6 * (c1 * c234 * c5 + s1 * s5) - s6 * c1 * s234;
      T(0, 1) = -s6 * (c1 * c234 * c5 + s1 * s5) - c6 * c1 * s234;
      T(0, 2) = c1 * c234 * s5 - s1 * c5;

      T(1, 0) = c6 * (s1 * c234 * c5 - c1 * s5) - s6 * s1 * s234;
      T(1, 1) = -s6 * (s1 * c234 * c5 - c1 * s5) - c6 * s1 * s234;
      T(1, 2) = s1 * c234 * s5 + c1 * c5;

      T(2, 0) = -c6 * s234 * c5 - s6 * c234;
      T(2, 1) = s6 * s234 * c5 - c6 * c234;
      T(2, 2) = -s234 * s5;

      Eigen::Affine3d result;
      result.matrix() = T;
      return result;
    }
    Eigen::Matrix<double, 6, 6> OperationalSpaceController6DOF::computeJacobian(const Vector6d &q)
    {
      // Compute geometric Jacobian for UR10 robot numerically
      // Using numerical differentiation for robustness
      Eigen::Matrix<double, 6, 6> J;
      J.setZero();

      const double eps = 1e-7;

      // Get current end-effector pose
      Eigen::Affine3d T0 = forwardKinematics(q);
      Vector3d p0 = T0.translation();

      for (int i = 0; i < 6; ++i)
      {
        Vector6d q_plus = q;
        q_plus(i) += eps;

        Eigen::Affine3d T_plus = forwardKinematics(q_plus);
        Vector3d p_plus = T_plus.translation();

        // Linear velocity Jacobian (numerical derivative)
        J.block<3, 1>(0, i) = (p_plus - p0) / eps;

        // Angular velocity Jacobian (z-axis of each joint in base frame)
        // For revolute joints: omega = z_i * q_dot_i
        // This is a simplified approach - computing z-axis rotation contribution
        Eigen::Matrix3d R_diff = T_plus.rotation() * T0.rotation().transpose();
        Eigen::AngleAxisd aa(R_diff);
        J.block<3, 1>(3, i) = aa.axis() * aa.angle() / eps;
      }

      return J;
    }
    Vector3d OperationalSpaceController6DOF::getEndEffectorPosition(const Vector6d &q)
    {
      Eigen::Affine3d T = forwardKinematics(q);
      return T.translation();
    }
    Vector3d OperationalSpaceController6DOF::getEndEffectorVelocity(const Vector6d &q, const Vector6d &qp)
    {
      // xdot = J * qdot (linear velocity part only)
      Eigen::Matrix<double, 6, 6> J = computeJacobian(q);
      Eigen::Matrix<double, 3, 6> J_linear = J.block<3, 6>(0, 0);
      return J_linear * qp;
    }

  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli
