#include <tum_ics_ur10_controller_tutorial/operational_space_controller_6dof.h>
#include <tum_ics_ur_robot_msgs/ControlData.h>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <nav_msgs/Path.h>
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
      Kp_PD_ = 5.0 * Matrix3d::Identity();
      Kp_PD_ << 2.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 10.0;
      Kd_PD_ = 10.0 * Matrix3d::Identity();
      Kreg_PD_ = Matrix6d::Zero();
      Kreg_PD_.block<3, 3>(0, 0) = 40.0 * Matrix3d::Identity();
      Kreg_PD_.block<3, 3>(3, 3) = 5 * Matrix3d::Identity();

      Kreg_PD_cart_ = Matrix3d::Zero();
      Kreg_PD_cart_.block<3, 3>(0, 0) << 500, 0.0, 0.0,
                                        0.0, 500, 0.0,
                                        0.0, 0.0, 500;
      // Kreg_PD_cart_.block<3, 3>(3, 3) = 5.0 * Matrix3d::Identity();
      Kp_PID_ = 5.0 * Matrix3d::Identity();
      Kd_PID_ = 5.0 * Matrix3d::Identity();
      Ki_PID_ = 5.0 * Matrix3d::Identity();

      // Trajectory visualization (RViz)
      traj_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/desired_trajectory_pose", 10);
      traj_path_pub_ = nh_.advertise<nav_msgs::Path>("/desired_trajectory_path", 1, true);
      traj_path_msg_.header.frame_id = "world";
      traj_max_points_ = 2000;

      // Trajectory visualization (RViz) for weld path
      weld_traj_pub_ = nh_.advertise<nav_msgs::Path>("/weld_trajectory_path", 1, true);
      weld_traj_path_msg_.header.frame_id = "world";
      weld_traj_max_points_ = 2000;
      weld_path_idx_ = 0;

      // Initialize time parameters (matching Python)
      t_INIT_ = 15.0;    // Was 5.0, Python uses 10.0
      t_LINEAR_ = 20.0;
      t_TRACK_SLOW_ = 100.0;
      t_TRACK_FAST_ = 0.0;  // Match Python's 40s
      q0_ << 0.0, -M_PI / 2.0, 0.0, -M_PI / 2.0, 0, 0.0; // Starting pose for trajectory
      
      
      // Initialize poses
      start_track_pose_ << 0.0, -M_PI / 3.0, -M_PI + M_PI / 2.0, 0, 0.0, 0.0;
      non_singular_pose_ << 0.0, -M_PI / 2.0, -M_PI / 2, -M_PI / 2, M_PI / 2, 0.0;  // Match Python
      // test good starting pose for following linear motion: 0, -90, -90, -90, 90, 0;
      //non_singular_pose_ << 0, -M_PI/2, -M_PI/2, -M_PI/2, M_PI/2, 0;
      
      // Initialize circular trajectory parameters
      orbital_center_.setZero();
      circle_radius_ = 0.15;      // Smaller radius for safer motion
      vertical_amplitude_ = 0.02; // Small Z oscillation

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
      admittance_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/admittance_state", 10);
      tf_pub_ = nh_.advertise<tf2_msgs::TFMessage>("/tf", 100);
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
      // Initialize admittance filter
      admittance_filter_ = std::make_unique<AdmittanceFilter>(nh_, "/schunk_netbox/raw");
      use_admittance_ = true;  // Disabled by default, enable via parameter

      // subscribe to nav_msgs/Path "weld_path"
      weld_path_sub_ = nh_.subscribe("/weld_line_vision/weld_path", 1, &OperationalSpaceController6DOF::weldPathCallback, this);
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
                      << ", TRACK_SLOW: " << t_TRACK_SLOW_ << ", TRACK_FAST: " << t_TRACK_FAST_ << ", non_singular: " << non_singular_pose_.transpose());

      // Initialize admittance filter
      ros::param::get(ns + "/use_admittance", use_admittance_);
      if (use_admittance_)
      {
        admittance_filter_->init("~admittance");
        ROS_WARN_STREAM("Admittance control ENABLED");
      }
      else
      {
        ROS_INFO_STREAM("Admittance control disabled");
      }

      return true;
    }
    bool OperationalSpaceController6DOF::start()
    {
      ROS_WARN_STREAM("OperationalSpaceController6DOF::start");
      return true;
    }
    Vector6d OperationalSpaceController6DOF::update(const RobotTime &time, const JointState &state)
    {
      JointState state_adj = state;
      Vector6d offset;
      //offset << 0,  M_PI/2, 0, M_PI/2, 0, 0;
      offset << 0, 0, 0, 0, 0, 0;

      //state_adj.q += offset;
      // Vector6d offset;
      // offset << 0,  M_PI/2, 0, M_PI/2, 0, 0;
      // state_adj.q +=  offset;
      // First iteration initialization
      if (is_first_iter_)
      {
        q_start_ = state_adj.q;
        t0_ = time.tD();
        last_t_ = t0_;
        last_q_ = state_adj.q;
        // Get initial Cartesian position using our FK
        X_ = getEndEffectorPosition(state_adj.q);
        last_X_ = X_;

        ROS_WARN_STREAM("START [RAD]: \n" << q_start_.transpose());
        ROS_WARN_STREAM("Initial EE position X_: " << X_.transpose() << " (should be ~0.5-1.0m values)");
        
        // Also verify track position FK
        Vector3d track_pos = getEndEffectorPosition(start_track_pose_);
        ROS_WARN_STREAM("Track pose FK: " << track_pos.transpose());
        
        is_first_iter_ = false;
      }

      // Update current state
      q_ = state_adj.q;
      //q_ -= [0, M_PI/2, 0, M_PI/2, 0, 0];
      X_ = getEndEffectorPosition(state.q);
      
      publishTF_raw("world", "ee_link", X_(0), X_(1), X_(2));
      // Calculate elapsed time
      double t_elapsed = time.tD() - t0_;
      double dt = time.tD() - dt_acc_; // Assume 500Hz control loop (adjust as needed)
      dt_acc_ = time.tD();

      // Update state machine
      updateStateMachine(t_elapsed);

      // Check for singularity (manipulability)
      if (control_space_ == ControlSpace::CARTESIAN)
      {
        Eigen::Matrix<double, 6, 6> J = computeJacobian(state_adj.q);
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

        // Publish desired trajectory for RViz visualization
        if (current_state_ != State::INIT)
        {
          geometry_msgs::PoseStamped pose_msg;
          pose_msg.header.stamp = ros::Time::now();
          pose_msg.header.frame_id = traj_path_msg_.header.frame_id;
          pose_msg.pose.position.x = x_des(0);
          pose_msg.pose.position.y = x_des(1);
          pose_msg.pose.position.z = x_des(2);
          pose_msg.pose.orientation.w = 1.0;
          pose_msg.pose.orientation.x = 0.0;
          pose_msg.pose.orientation.y = 0.0;
          pose_msg.pose.orientation.z = 0.0;

          traj_path_msg_.header.stamp = pose_msg.header.stamp;
          traj_path_msg_.poses.push_back(pose_msg);
          if (traj_path_msg_.poses.size() > traj_max_points_)
          {
            traj_path_msg_.poses.erase(traj_path_msg_.poses.begin());
          }

          traj_pose_pub_.publish(pose_msg);
          traj_path_pub_.publish(traj_path_msg_);

          // Also publish weld trajectory if we have it
          if (!weld_traj_path_msg_.poses.empty())
          { 
            weld_traj_pub_.publish(weld_traj_path_msg_);
          }
        }


      // Compute control torque
      Vector6d tau;
      tau.setZero();
      
      Vector3d xdot_current = getEndEffectorVelocity(state_adj.q, state_adj.qp);
      std_msgs::Float32MultiArray x_msg;
      x_msg.data = {static_cast<float>(xdot_current(0)), 
                    static_cast<float>(xdot_current(1)), 
                    static_cast<float>(xdot_current(2))};
      cartesian_pub_.publish(x_msg);
      if (control_space_ == ControlSpace::JOINT)
      {
        tau = jointSpaceController(state_adj, q_des, qdot_des, qddot_des, dt);
        X_ = getEndEffectorPosition(q_);
      }
      else // CARTESIAN
      {
        Vector3d xdot_current = getEndEffectorVelocity(state_adj.q, state_adj.qp);
        
        // DEBUG: Log x_des to verify it's not zero
        ROS_INFO_STREAM_THROTTLE(0.5, "[CART] x_des=" << x_des.transpose() 
                                 << " X_=" << X_.transpose()
                                 << " delta=" << (X_ - x_des).transpose());
        X_ = getEndEffectorPosition(q_);
        tau = cartesianSpaceController(state_adj, X_, xdot_current, x_des, xdot_des, xddot_des, dt);
      }

      // Publish debug data
      tum_ics_ur_robot_msgs::ControlData msg;
      msg.header.stamp = ros::Time::now();
      msg.time = time.tD();
      for (int i = 0; i < STD_DOF; i++)
      {
        msg.q[i] = state_adj.q(i);
        msg.qp[i] = state_adj.qp(i);
        msg.qpp[i] = state_adj.qpp(i);
        msg.qd[i] = q_des(i);
        msg.qpd[i] = qdot_des(i);
        msg.Dq[i] = state_adj.q(i) - q_des(i);
        msg.Dqp[i] = state_adj.qp(i) - qdot_des(i);
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
        //last_X_(2) -= admittance_filter_->update(0, last_q_)(2);
      }
      else if (t_elapsed > t_INIT_ && current_state_ == State::INIT)
      {
        new_state = State::LINEAR;
        control_space_ = ControlSpace::CARTESIAN;
        //last_t_ = t_elapsed + t0_;
        last_t_ = ros::Time::now().toSec();
        last_q_ = q_;
        last_X_ = X_;
      }

      // If state changed, update transition variables
      if (new_state != current_state_)
      {
        ROS_INFO_STREAM("State transition: " << static_cast<int>(current_state_) 
                        << " -> " << static_cast<int>(new_state));
        
        current_state_ = new_state;
        // FIXED: Use consistent time source! t_elapsed is based on time.tD(), not ros::Time::now()
        // We need to store the controller time at transition, not wall clock time
        last_t_ = t_elapsed + t0_;  // Convert back to absolute controller time
        last_q_ = q_;
        //last_X_ = X_;
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
      // end effector height = 49-20 = 29cm, difference between base and tabel = table 8cm lower
      track_pos_cartesian << -0.65, 0.0, 0.29-0.09-0.045; // Override with fixed position for testing
      // set first waypoint of weld path to track_pos_cartesian 
      if (!weld_traj_path_msg_.poses.empty())
      {
        track_pos_cartesian << weld_traj_path_msg_.poses[0].pose.position.x, 
                              weld_traj_path_msg_.poses[0].pose.position.y, 
                              weld_traj_path_msg_.poses[0].pose.position.z;
      }
      double omega = 2.0 * M_PI / t_TRACK_SLOW_;
      double omega_z = omega * 15.0;
      double omega_fast = 2.0 * M_PI / t_TRACK_FAST_;
      double omega_z_fast = omega_fast * 15.0;

      omega *= 4.0;
      omega_z *= 4.0;
      omega_fast *= 4.0;
      omega_z_fast *= 4.0;

      // FIXED: offset should be time since current state started, not absolute time
      // Python uses: tau = (t_now - self.phase_entry_time).to_sec()
      double offset = t - last_t_;
      
      // DEBUG: Log trajectory generation details
      ROS_INFO_STREAM_THROTTLE(0.5, "[TRAJ] State=" << static_cast<int>(current_state_) 
                               << " t=" << t << " last_t_=" << last_t_ << " offset=" << offset
                               << " last_X_=" << last_X_.transpose()
                               << " track_pos=" << track_pos_cartesian.transpose()
                              << "time offset" << offset);

      switch (current_state_)
      {
      case State::INIT:
        // Linear interpolation to non-singular pose in joint space
        q_des = q0_ + ((non_singular_pose_ - q0_) / t_INIT_) * offset;
        break;

      case State::LINEAR:
      {
        // Linear interpolation in Cartesian space to tracking start position


        x_des = last_X_ + ((track_pos_cartesian - last_X_) / t_LINEAR_) * offset;
        //x_des(2) = last_X_(2); // Keep Z constant at desired valuelast_X_
        // NOTE: adding xdot_des here didnt change anything?
        xdot_des = (track_pos_cartesian - last_X_) / t_LINEAR_;
        //xdot_des(2) = 0; // No velocity in Z direction during linear phase
        break;
      }
      //ROS_INFO_STREAM_THROTTLE(0.5, "track_pos_cartesian - last_X_) / t_LINEAR_" << ((track_pos_cartesian - last_X_) / t_LINEAR_));

      case State::TRACK_SLOW:
      {

        // Follow weld path waypoints from subscribed nav_msgs/Path
        if (weld_traj_path_msg_.poses.empty())
        {
          // No path received yet - hold current position
          ROS_WARN_STREAM_THROTTLE(2.0, "[TRACK] No weld path received yet, holding position");
          x_des = last_X_;
          xdot_des.setZero();
          break;
        }

        std::size_t n_poses = weld_traj_path_msg_.poses.size();

        // Compute time per segment: spread t_TRACK_SLOW_ evenly across all segments
        double t_per_segment = t_TRACK_SLOW_ / static_cast<double>(n_poses > 1 ? n_poses - 1 : 1);

        // Determine which segment we're on based on elapsed time
        std::size_t seg_idx = static_cast<std::size_t>(offset / t_per_segment);
        
        if (seg_idx >= n_poses - 1)
        {
          ROS_WARN_STREAM_THROTTLE(2.0, "[TRACK] Completed all waypoints, holding last position");
          // Clamp to last waypoint
          seg_idx = n_poses - 1;
          const auto& p = weld_traj_path_msg_.poses[seg_idx].pose.position;
          x_des << p.x, p.y, p.z;
          xdot_des.setZero();
        }
        else
        {
          // Interpolate between waypoint seg_idx and seg_idx+1
          double seg_offset = offset - seg_idx * t_per_segment;
          double alpha = seg_offset / t_per_segment;  // 0..1 within segment

          const auto& p0 = weld_traj_path_msg_.poses[seg_idx].pose.position;
          const auto& p1 = weld_traj_path_msg_.poses[seg_idx + 1].pose.position;

          Vector3d wp0(p0.x, p0.y, p0.z);
          Vector3d wp1(p1.x, p1.y, p1.z);

          x_des = wp0 + alpha * (wp1 - wp0);
          xdot_des = (wp1 - wp0) / t_per_segment; 
          // test if x_des is part of weld path and increase seg_idx if so
          if ((x_des - wp1).norm() < 0.005) // If we're within 1cm of the next waypoint, move to the next segment
          {
            seg_idx++;
          }
        }

        weld_path_idx_ = seg_idx;  // Store for debugging
        ROS_INFO_STREAM_THROTTLE(1.0, "[TRACK] seg=" << seg_idx << "/" << n_poses - 1
                                 << " x_des=" << x_des.transpose());
                                
        break;
      }

      case State::TRACK_FAST:
      {
        // Same continuous trajectory but faster
        double theta = omega_fast * offset;
        
        x_des(0) = orbital_center_(0) ;//+ circle_radius_ * std::cos(theta);
        x_des(1) = orbital_center_(1);// + circle_radius_ * std::sin(theta);
        //x_des(2) = orbital_center_(2) + vertical_amplitude_ * std::sin(omega_z_fast * theta);
        //x_des(2) = orbital_center_(2);

        xdot_des(0) =0 ;// -circle_radius_ * omega_fast * std::sin(theta);
        xdot_des(1) = 0;//circle_radius_ * omega_fast * std::cos(theta);
        //xdot_des(2) = vertical_amplitude_ * omega_z_fast * std::cos(omega_z_fast * theta);
        xdot_des(2) = 0;
        break;
      }

      case State::END:
      default:
        q_des = closestEquivalentAngles(non_singular_pose_, q_);
        break;
      }
    }
    // this one should normally come from the adaptive controller, but 
    Vector6d OperationalSpaceController6DOF::jointSpaceController(const JointState &state,
                                                                   const Vector6d &q_des,
                                                                   const Vector6d &qdot_des,
                                                                   const Vector6d &qddot_des,
                                                                   double dt)
    {

      // std_msgs::Float32MultiArray x_des_msg;
      // x_des_msg.data = {static_cast<float>(q_des(0)), 
      //                   static_cast<float>(q_des(1)), 
      //                   static_cast<float>(q_des(2))};
      // joint_states_des_pub_.publish(x_des_msg);
      Vector6d tau;
      tau.setZero();
      Eigen::Matrix<double, 6, 6> Kp = (10 * Eigen::Matrix<double, 6, 6>::Identity());
      // Errors
      Vector6d delta_q = state.q - q_des;
      Vector6d delta_qdot = qdot_des - Kp * delta_q;

      // Reference velocity with proportional term
      Vector6d qrdot = qdot_des - Kp * delta_q; // Normalize by Kreg scale

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

      // ======================================================================
      // ADMITTANCE CONTROL INTEGRATION
      // ======================================================================
      // The admittance filter computes a position offset based on external
      // forces measured by the FT sensor.
      //
      // Modified desired trajectory:
      //
      //     x_des_modified   = x_des_trajectory + x_admittance
      //     xdot_des_modified = xdot_des_trajectory + xdot_admittance
      //
      // This allows the robot to:
      //   1. Follow the planned trajectory (x_des_trajectory)
      //   2. Comply with external forces (x_admittance offset)
      //
      // For end effector drawing:
      //   - X,Y follow trajectory exactly (x_admittance.xy = 0)
      //   - Z adjusts to maintain constant contact force
      // ======================================================================
      Vector3d x_des_modified = x_des;
      Vector3d xdot_des_modified = xdot_des;
      Vector3d xddot_des_modified = xddot_des;
      Eigen::Matrix<double, 3, 3> selectionMatrixAdmittance = Matrix3d::Identity();
      // selectionMatrixAdmittance << 1, 0, 0,
      //                              0, 1, 0,
      //                              0, 0, 1;
      if (use_admittance_ && admittance_filter_)
      {
        // Debug: Check joint configuration before passing to admittance filter
        bool joints_valid = true;
        for (int i = 0; i < 6; i++)
        {
          if (!std::isfinite(state.q(i)))
          {
            ROS_ERROR_STREAM_THROTTLE(0.5, "[CART CTRL] Joint " << i << " is NaN/Inf: " << state.q(i));
            joints_valid = false;
          }
        }
        
        if (!joints_valid)
        {
          ROS_ERROR_STREAM_THROTTLE(0.5, "[CART CTRL] Invalid joint config, skipping admittance. q = " << state.q.transpose());
        }
        else
        {
          // Get admittance offset: x_a from M*x_ddot + D*x_dot + K*x = F_ext - F_des
          Vector3d x_offset = admittance_filter_->update(dt, state.q);
          Vector3d xdot_offset = admittance_filter_->getVelocityOffset();
          Vector3d xddot_offset = admittance_filter_->getAccelerationOffset();

          // Modify desired trajectory with admittance offset
          x_des_modified = x_des + x_offset;//selectionMatrixAdmittance * x_offset;
          xdot_des_modified = xdot_des + xdot_offset;//selectionMatrixAdmittance * xdot_offset;
          // NOTE: Adding xddot_des_modified let joint velocities explode
          xddot_des_modified = xddot_des + xddot_offset;//selectionMatrixAdmittance * xddot_offset;
          ROS_INFO_STREAM_THROTTLE(0.5, "[ADMITTANCE] x_offset=" << x_offset.transpose() 
                                   << " xdot_offset=" << xdot_offset.transpose() 
                                   << " xddot_offset=" << xddot_offset.transpose());

          // Publish admittance state for debugging
          std_msgs::Float32MultiArray adm_msg;
          Vector3d F_ext = admittance_filter_->getExternalForce();
          adm_msg.data = {
            static_cast<float>(x_offset(0)),
            static_cast<float>(x_offset(1)),
            static_cast<float>(x_offset(2)),
            static_cast<float>(F_ext(0)),
            static_cast<float>(F_ext(1)),
            static_cast<float>(F_ext(2)), 
            static_cast<float>(xddot_offset(0)),
            static_cast<float>(xddot_offset(1)),
            static_cast<float>(xddot_offset(2))
          };
          admittance_pub_.publish(adm_msg);
        }
        
      }
      
      
      ROS_INFO_STREAM_THROTTLE(0.5,  "[CART CTRL] x_des=" << x_des.transpose()
                                  << " xdot_des=" << xdot_des.transpose()
                                  << " xddot_des=" << xddot_des.transpose()
                                  <<  " x_des_modified=" << x_des_modified.transpose() 
                                  << " xdot_des_modified=" << xdot_des_modified.transpose() 
                                  << " xddot_des_modified=" << xddot_des_modified.transpose()
                                  << " x_current=" << x_current.transpose() 
                                  << " xdot_current=" << xdot_current.transpose())
                                  ;
      // Cartesian errors (use modified desired if admittance is enabled)
      Vector3d delta_x = x_current - x_des_modified;
      // ROS_WARN_STREAM("delta_x: " << delta_x.transpose());
      Vector3d delta_xdot = xdot_current - xdot_des_modified;

      // Update integral error
      integral_error_acc_ += delta_x * dt;

      // Get full Jacobian and extract translational part (rows 0-2)
      Eigen::Matrix<double, 6, 6> J_full = computeJacobian(state.q);
      // NOTE: Robot gets unstable, if we use (3,0) -> hence (3,0) is the rotational part and (0,0) is the translational part of the Jacobian
      Eigen::Matrix<double, 3, 3> J_ee = J_full.block<3, 3>(0, 0);  // Translational Jacobian

      // Compute pseudoinverse of Jacobian
      Eigen::MatrixXd J_pinv = pseudoInverse(J_ee);

      Eigen::Matrix<double,6,6> Jdot_ee_w  = computeJacobianDotTranslational(state.q, state.qp);
      Eigen::Matrix<double, 3, 3> Jdot_ee_w_translation = Jdot_ee_w.block<3, 3>(0, 0); // Get translational part of Jacobian derivative in world frame
      //PD controller seems to be correctly
      Eigen::Matrix<double, 6, 6> selectionMatrix = Matrix6d::Zero();
      selectionMatrix << 1, 0, 0, 0, 0, 0,
                          0, 1, 0, 0, 0, 0,
                          0, 0, 1, 0, 0, 0,
                          0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0;
      if (controller_type_ == ControllerType::PD)
      {
      Vector6d qrdot, qrddot;
      qrdot.setZero();
      qrddot.setZero();
      // Reference Cartesian velocityKreg_PD_cart_
      Vector3d xrdot = xdot_des_modified - Kp_PD_ * delta_x;
      Vector3d xrddot = xddot_des_modified - delta_xdot;//Kp_PD_ * delta_xdot;
      // Map to joint space
      qrdot.head(3) = J_pinv * xrdot;
      qrddot.head(3) = J_pinv * (xrddot- Jdot_ee_w_translation * qrdot.head(3)); // Jdot_ee is the time derivative of the Jacobian 

      // Sliding variable in joint space
      Vector3d S_q = state.qp.head(3) - qrdot.head(3);
      
      std_msgs::Float32MultiArray delta_msg;
      delta_msg.data = {static_cast<float>(xrdot(0)), 
                        static_cast<float>(xrdot(1)), 
                        static_cast<float>(xrdot(2))};
      steady_state_error_pub_.publish(delta_msg);

      std_msgs::Float32MultiArray x_des_msg;
      x_des_msg.data = {static_cast<float>(x_des(0)),
                        static_cast<float>(x_des(1)),
                        static_cast<float>(x_des(2))};
      joint_states_des_pub_.publish(x_des_msg);
        //TODO: implement missing parts aka ro, computeRegressor and
        //Vector6d Yr = computeRegressor(state.q, state.qp, qrdot, xddot_des); // Compute regressor based on current state and desired acceleration
        //Vector6d theta -= (ro * (Yr.transpose() * S_q)) * dt; // Update adaptive parameters using gradient descent
        //Yr_theta = computeYr_theta(state.q, state.qp, qrdot, qrddot); 
        // Simple PD control (can be extended with adaptive control)
        tau.head(3) = -Kreg_PD_cart_ * S_q ;//+ Yr_theta;
        tau(3) = jointControl(state.q, 50);
        tau(4) = (M_PI/2 - state.q(4)) * 10;
        tau += selectionMatrix * computeReferenceTorques(state.q, state.qp, qrdot, qrddot); // Add feedforward term based on desired trajectory
      }
      else if (controller_type_ == ControllerType::PID)
      {
        
        // //TODO: implement correct version of PID Controller 
        // // PID in Cartesian space
        // Vector3d xrdot = xdot_des - Kp_PID_ * delta_x - Ki_PID_ * integral_error_acc_;
        // Vector3d xrddot = xddot_des - Kp_PID_ * delta_xdot - Ki_PID_ * delta_x;
        // Vector6d qrdot = J_pinv * xrdot;
        // // TODO: implement Jdot_ee helper function that calculates the time derivative of the Jacobian based on current q and qp
        // Vector6d qrddot = J_pinv * (xrddot- Jdot_ee_w_translation * qrdot); // Jdot_ee is the time derivative of the Jacobian 
        // Vector6d S_q = state.qp - qrdot;
        // //TODO: add helper function that calculate M, C, G and maybe also regressor Y from model.zip files
        // //Yr_theta = M * qrddot + C * qrdot + G; // Regressor-based adaptive control law (simplified)
        // Yr_theta = computeYr_theta(state.q, state.qp, qrdot, qrddot);
        // tau = -Kd_PID_(0, 0) * S_q + Yr_theta;  // Use scalar Kd for simplicity
      }

      // Publish Cartesian tracking data

      std_msgs::Float32MultiArray x_msg;
      x_msg.data = {static_cast<float>(xdot_current(0)), 
                    static_cast<float>(xdot_current(1)), 
                    static_cast<float>(xdot_current(2))};
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

    void OperationalSpaceController6DOF::publishTF_raw(const std::string& parent,
                   const std::string& child,
                   double x, double y, double z)
    {
        geometry_msgs::TransformStamped t;

        t.header.stamp = ros::Time::now();
        t.header.frame_id = parent;
        t.child_frame_id = child;

        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = z;

        t.transform.rotation.w = 1.0; // identity rotation

        tf2_msgs::TFMessage msg;
        msg.transforms.push_back(t);
        tf_pub_.publish(msg);
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
      Eigen::Matrix<double, 6, 1> V_ee_w = J * qp; // End-effector twist in world frame
      return V_ee_w.block<3, 1>(0, 0); // 0,0 for linear velocity -> had here 3,0 before!!!

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

    Vector6d OperationalSpaceController6DOF::computeReferenceTorques(const Vector6d &q, const Vector6d &qdot, Vector6d &qdotr, Vector6d &qddotr)
    {
      return model_.inertiaMatrix(q) * qddotr + model_.centripetalMatrix(q, qdot) * qdotr + model_.gravityVector(q);
    }
    double OperationalSpaceController6DOF::jointControl(const Vector6d &q, double gain) 
    {
      Vector3d z;
      Vector3d x;
      z.setZero();
      x.setZero();
      x << 1, 0, 0;
      z << 0, 0, 1;
      double dot = z.transpose() * model_.T_5_w_inv(q).matrix().block<3, 3>(0, 0) * x;
      return -dot * gain;
    }
    
    // &OperationalSpaceController6DOF::weldPathCallback for reading out the path once and then unsubscribing, since we only need the path for visualization and not for control
    void OperationalSpaceController6DOF::weldPathCallback(const nav_msgs::Path::ConstPtr& msg)
    {      ROS_INFO_STREAM("Received weld path with " << msg->poses.size() << " poses");
      if (msg->poses.empty())
      {
        ROS_WARN_STREAM("Received empty weld path, ignoring");
        return;
      }
      // Store path in global variable for trajectory generation and visualization
      weld_traj_path_msg_ = *msg;
      
      // transform work_path to world frame using T_world_work
      // Get Transform from workspace_frame to world frame using tf2 -> is zero?
      geometry_msgs::TransformStamped T_world_work = tf_buffer_.lookupTransform("world", "workspace_frame", ros::Time(0));
      ROS_INFO_STREAM("Transform from workspace_frame to world: translation = (" 
                      << T_world_work.transform.translation.x << ", "
                      << T_world_work.transform.translation.y << ", "
                      << T_world_work.transform.translation.z << ")");
      //T_world_path = T_world_work * T_work_path;

      // transform all poses from workspace frame to world frame
      for (size_t i = 0; i < weld_traj_path_msg_.poses.size(); i++)
      {
        geometry_msgs::PoseStamped& ps = weld_traj_path_msg_.poses[i];
        geometry_msgs::PoseStamped ps_world;
        ps_world.header.frame_id = "world";
        // ps_world.pose.position.x = 0.7 + ps.pose.position.x; //T_world_work.transform.translation.x + ps.pose.position.x;
        // ps_world.pose.position.y = 0.125 + ps.pose.position.y;  //T_world_work.transform.translation.y + ps.pose.position.y; 
        // ps_world.pose.position.z = 0.06 + ps.pose.position.z;//T_world_work.transform.translation.z + ps.pose.position.z;
        ps_world.pose.position.x = T_world_work.transform.translation.x + ps.pose.position.x;
        ps_world.pose.position.y = T_world_work.transform.translation.y + ps.pose.position.y; 
        ps_world.pose.position.z = T_world_work.transform.translation.z + ps.pose.position.z;

        ps_world.pose.orientation = ps.pose.orientation; // Assuming no rotation between frames for simplicity
        weld_traj_path_msg_.poses[i] = ps_world;
      }
      //print out all poses for debugging
      for (size_t i = 0; i < weld_traj_path_msg_.poses.size(); i++)
      {
        const auto& p = weld_traj_path_msg_.poses[i].pose.position;
        ROS_INFO_STREAM("Pose " << i << ": (" << p.x << ", " << p.y << ", " << p.z << ")");
      }
        
      // Unsubscribe after receiving the path once
      weld_path_sub_.shutdown();
    }
  
  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli
