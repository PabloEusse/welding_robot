#include <tum_ics_ur10_controller_tutorial/AdmittanceFilter.h>
#include <algorithm>

namespace tum_ics_ur_robot_lli
{

AdmittanceFilter::AdmittanceFilter(ros::NodeHandle& nh, const std::string& ft_topic)
    : nh_(nh),
      sensor_connected_(false),
      model_("ur10_model")
{
  // Initialize vectors
  F_ext_.setZero();
  T_ext_.setZero();
  F_desired_.setZero();
  F_desired_ << 0.0, 0.0, 3;
  x_.setZero();
  x_dot_.setZero();
  x_ddot_.setZero();
  ROS_WARN_STREAM("axis modes");
  // Default admittance parameters (tuned for end effector drawing)
  // Z-axis: compliant for force control
  // X,Y: stiff (position control mode by default)
  M_d_ << 0.0, 0.0, 5;    // Virtual mass [kg]
  D_d_ << 1000.0, 1000.0, 350.0;  // Virtual damping [Ns/m] - with 60 = xddot_offset = 2.3683 at state transition, with 70 = 10^-6
  K_d_ << 5.0, 5.0, 15;    // Virtual stiffness [N/m] - 0 for pure force tracking - activate for testing

  // Default: X,Y position control, Z force control
  axis_modes_ = {AxisMode::POSITION, AxisMode::POSITION, AxisMode::FORCE};
  setAxisMode(3, axis_modes_[3]);
  // Safety limits
  x_max_ << 0.02, 0.02, 0.5;       // Max 5cm offset
  x_dot_max_ << 0.1, 0.1, 0.5;      // Max 10cm/s velocity

  // Subscribe to FT sensor
  ft_sub_ = nh_.subscribe(ft_topic, 1, &AdmittanceFilter::ftSensorCallback, this);

  ROS_INFO_STREAM("AdmittanceFilter: Subscribing to " << ft_topic);
}

bool AdmittanceFilter::init(const std::string& ns)
{
  if (!ros::param::has(ns))
  {
    ROS_WARN_STREAM("AdmittanceFilter: No parameters found at " << ns << ", using defaults");
    return true;
  }

  std::vector<double> vec;

  // Virtual mass
  if (ros::param::get(ns + "/mass", vec) && vec.size() >= 3)
  {
    M_d_ << vec[0], vec[1], vec[2];
  }

  // Virtual damping
  if (ros::param::get(ns + "/damping", vec) && vec.size() >= 3)
  {
    D_d_ << vec[0], vec[1], vec[2];
  }

  // Virtual stiffness
  if (ros::param::get(ns + "/stiffness", vec) && vec.size() >= 3)
  {
    K_d_ << vec[0], vec[1], vec[2];
  }

  // Desired force
  if (ros::param::get(ns + "/desired_force", vec) && vec.size() >= 3)
  {
    F_desired_ << vec[0], vec[1], vec[2];
  }

  // Position limits
  if (ros::param::get(ns + "/max_offset", vec) && vec.size() >= 3)
  {
    x_max_ << vec[0], vec[1], vec[2];
  }

  // Velocity limits
  if (ros::param::get(ns + "/max_velocity", vec) && vec.size() >= 3)
  {
    x_dot_max_ << vec[0], vec[1], vec[2];
  }

  ROS_INFO_STREAM("AdmittanceFilter initialized:");
  ROS_INFO_STREAM("  M_d: " << M_d_.transpose());
  ROS_INFO_STREAM("  D_d: " << D_d_.transpose());
  ROS_INFO_STREAM("  K_d: " << K_d_.transpose());
  ROS_INFO_STREAM("  F_desired: " << F_desired_.transpose());

  return true;
}

void AdmittanceFilter::reset()
{
  x_.setZero();
  x_dot_.setZero();
  x_ddot_.setZero();
  ROS_INFO("AdmittanceFilter: State reset");
}

Vector3d AdmittanceFilter::update(double dt, const cc::JointPosition& q)
{
  if (dt <= 0.0 || dt > 0.1)
  {
    ROS_WARN_THROTTLE(1.0, "AdmittanceFilter: Invalid dt=%.4f", dt);
    return x_;
  }

  // Get current force (thread-safe)
  Vector3d F_ext;
  {
    std::lock_guard<std::mutex> lock(ft_mutex_);
    F_ext = F_ext_;
    // set F_ext to some test value if sensor is not connected for testing without hardware
    if (!sensor_connected_)
    {
      F_ext << 0.0, 0.0, -50.0; // Simulate constant 5N downward force for testing
    }
  }

  // Check sensor timeout (1 second)
  if (sensor_connected_ && (ros::Time::now() - last_ft_time_).toSec() > 1.0)
  {
    ROS_WARN_THROTTLE(1.0, "AdmittanceFilter: FT sensor timeout!");
    sensor_connected_ = false;
  }

  // Check joint configuration validity BEFORE using it
  bool q_valid = true;
  for (int i = 0; i < q.size(); i++)
  {
    if (!std::isfinite(q(i)))
    {
      ROS_ERROR_STREAM_THROTTLE(1.0, "AdmittanceFilter: Joint " << i << " is invalid: " << q(i));
      q_valid = false;
    }
  }
  
  if (!q_valid)
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "AdmittanceFilter: Invalid joint configuration, skipping update. q = " << q.transpose());
    return x_;  // Return current offset without updating
  }
  
  // transform F_ext form end effector frame to world frame using current robot state
  Eigen::Matrix4d T_ef_w_inv = model_.T_ef_w_inv(q).matrix();
  //ROS_INFO_STREAM_THROTTLE(0.5, "AdmittanceFilter: T_ef_w_inv:\n" << model_.T_ef_w_inv);
  
  // Check for NaN in transformation (indicates bad joint config or model issue)
  if (!T_ef_w_inv.allFinite())
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "AdmittanceFilter: T_ef_w_inv contains NaN! Joint config: " << q.transpose());
    ROS_ERROR_STREAM_THROTTLE(1.0, "T_ef_w_inv:\n" << T_ef_w_inv);
    return x_;  // Return current offset without updating
  }
  
  F_ext = T_ef_w_inv.block<3, 3>(0, 0) * F_ext;

  // ========================================================================
  // FORCE ERROR COMPUTATION
  // ========================================================================
  // F_error = F_ext - F_des
  //
  // For end effector drawing with F_des = [0, 0, -5N]:
  //   - If end effector pushes surface with 7N: F_ext.z = -7N, F_error.z = -7-(-5) = -2N
  //     -> Negative error means too much force -> robot should move UP
  //   - If end effector barely touches with 3N: F_ext.z = -3N, F_error.z = -3-(-5) = +2N
  //     -> Positive error means too little force -> robot should move DOWN
  // ========================================================================
  Vector3d F_error = F_ext - F_desired_;

  // ========================================================================
  // ADMITTANCE DYNAMICS
  // ========================================================================
  // From: M * x_ddot + D * x_dot + K * x = F_error
  //
  // Solve for acceleration:
  //
  //     x_ddot = (1/M) * (F_error - D * x_dot - K * x)
  //
  // Physical interpretation:
  //   - F_error drives the motion (force input)
  //   - D * x_dot provides viscous damping (prevents oscillation)
  //   - K * x provides spring-like return to equilibrium
  //   - M determines response speed (larger M = slower, more stable)
  // ========================================================================
  Vector3d x_ddot = Vector3d::Zero();  // Initialize to prevent NaN

  for (int i = 0; i < 3; i++)
  {
    if (axis_modes_[i] == AxisMode::FORCE)
    {
      // Force control via admittance
      if (M_d_(i) > 0.0)
      {
        x_ddot(i) = (F_error(i) - D_d_(i) * x_dot_(i) - K_d_(i) * x_(i)) / M_d_(i);
      }
      else
      {
        x_ddot(i) = 0.0;
      }
    }
    else
    {
      // Position control: no admittance offset
      x_ddot(i) = -D_d_(i) * x_dot_(i) - K_d_(i) * x_(i);  // Decay to zero
      if (M_d_(i) > 0.0)
      {
        x_ddot(i) /= M_d_(i);
      }
    }
  }

  // Store acceleration for external access
  x_ddot_ = x_ddot;

  // Check for NaN in outputs
  if (!x_ddot_.allFinite() || !x_dot_.allFinite() || !x_.allFinite())
  {
    ROS_ERROR_STREAM_THROTTLE(0.5, "AdmittanceFilter: NaN detected! "
                               << "x_ddot=" << x_ddot_.transpose()
                               << " x_dot=" << x_dot_.transpose()
                               << " x=" << x_.transpose()
                               << " F_error=" << F_error.transpose());
  }
  // ========================================================================
  // NUMERICAL INTEGRATION (Forward Euler)
  // ========================================================================
  // Velocity update:
  //     x_dot[k+1] = x_dot[k] + x_ddot[k] * dt
  //
  // Position update:
  //     x[k+1] = x[k] + x_dot[k+1] * dt
  //
  // Note: Euler method is O(dt) accurate. For better stability with
  // high gains, consider Runge-Kutta or semi-implicit Euler.
  // ========================================================================

  // Integrate velocity: x_dot[k+1] = x_dot[k] + x_ddot * dt
  x_dot_ += x_ddot * dt;

  // Clamp velocity for safety: |x_dot| <= x_dot_max
  for (int i = 0; i < 3; i++)
  {
    x_dot_(i) = std::max(-x_dot_max_(i), std::min(x_dot_(i), x_dot_max_(i)));
  }

  // Integrate position: x[k+1] = x[k] + x_dot[k+1] * dt
  x_ += x_dot_ * dt;

  // Clamp position for safety: |x| <= x_max
  for (int i = 0; i < 3; i++)
  {
    x_(i) = std::max(-x_max_(i), std::min(x_(i), x_max_(i)));
  }

  // Debug output
  ROS_DEBUG_STREAM_THROTTLE(0.5, "Admittance: F_ext=" << F_ext.transpose()
                            << " F_err=" << F_error.transpose()
                            << " x=" << x_.transpose());

  return x_;
}

void AdmittanceFilter::ftSensorCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(ft_mutex_);

  F_ext_(0) = msg->wrench.force.x;
  F_ext_(1) = msg->wrench.force.y;
  F_ext_(2) = msg->wrench.force.z;
  //F_ext_ = model_.T_ef_w_inv(q).matrix().block<3, 3>(0, 0) * F_ext_;
  T_ext_(0) = msg->wrench.torque.x;
  T_ext_(1) = msg->wrench.torque.y;
  T_ext_(2) = msg->wrench.torque.z;
  //T_ext_ = model_.T_ef_w_inv(q).matrix().block<3, 3>(0, 0) * T_ext_;
  last_ft_time_ = msg->header.stamp;
  sensor_connected_ = true;
}

void AdmittanceFilter::setAxisMode(int axis, AxisMode mode)
{
  if (axis >= 0 && axis < 3)
  {
    axis_modes_[axis] = mode;
    ROS_INFO_STREAM("AdmittanceFilter: Axis " << axis << " set to "
                    << (mode == AxisMode::FORCE ? "FORCE" : "POSITION") << " mode");
  }
}

void AdmittanceFilter::setMass(const Vector3d& M_d)
{
  M_d_ = M_d;
}

void AdmittanceFilter::setDamping(const Vector3d& D_d)
{
  D_d_ = D_d;
}

void AdmittanceFilter::setStiffness(const Vector3d& K_d)
{
  K_d_ = K_d;
}

Vector3d AdmittanceFilter::getExternalForce() const
{
  std::lock_guard<std::mutex> lock(ft_mutex_);
  return F_ext_;
}
// Vector3d AdmittanceFilter::getVelocityOffset() const
// {
//   return x_dot_;
// }
Vector3d AdmittanceFilter::getAccelerationOffset() const
{
  return x_ddot_;
}


} // namespace tum_ics_ur_robot_lli
