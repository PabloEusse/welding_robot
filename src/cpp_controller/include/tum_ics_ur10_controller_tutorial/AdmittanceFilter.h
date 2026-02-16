#ifndef ADMITTANCE_FILTER_H
#define ADMITTANCE_FILTER_H

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <Eigen/Dense>
#include <mutex>
#include <ur_model/ur_model.h>
#include <control_core/types.h>

namespace tum_ics_ur_robot_lli
{

using Vector3d = Eigen::Matrix<double, 3, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix3d = Eigen::Matrix<double, 3, 3>;

/**
 * @brief Admittance Filter for force-controlled motion
 *
 * ============================================================================
 * ADMITTANCE CONTROL THEORY
 * ============================================================================
 *
 * Admittance control makes the robot behave like a virtual mass-spring-damper
 * system when interacting with the environment.
 *
 * Dynamic equation:
 *
 *     M_d * x_ddot + D_d * x_dot + K_d * x = F_ext - F_des
 *
 * Where:
 *   - x, x_dot, x_ddot : position, velocity, acceleration offset
 *   - M_d : virtual mass matrix [kg]        (inertia, larger = slower response)
 *   - D_d : virtual damping matrix [Ns/m]   (viscosity, larger = less oscillation)
 *   - K_d : virtual stiffness matrix [N/m]  (spring, 0 = pure force tracking)
 *   - F_ext : measured external force from FT sensor [N]
 *   - F_des : desired contact force [N]
 *
 * Rearranging for acceleration:
 *
 *     x_ddot = M_d^(-1) * (F_ext - F_des - D_d * x_dot - K_d * x)
 *
 * Discrete-time integration (Euler method):
 *
 *     x_dot[k+1] = x_dot[k] + x_ddot[k] * dt
 *     x[k+1]     = x[k]     + x_dot[k+1] * dt
 *
 * The resulting x is added to the desired trajectory:
 *
 *     x_des_modified = x_des_trajectory + x_admittance
 *
 * ============================================================================
 * USE CASE: End effector drawing with constant contact force
 * ============================================================================
 *   - X,Y axes: POSITION mode (pure trajectory following, no admittance)
 *   - Z axis:   FORCE mode (admittance active, maintains constant pressure)
 *
 * When F_ext.z > F_des.z (too much force), x.z increases (robot moves up)
 * When F_ext.z < F_des.z (too little force), x.z decreases (robot moves down)
 *
 */
class AdmittanceFilter
{
public:
  /**
   * @brief Control mode for each axis
   */
  enum class AxisMode
  {
    POSITION,  // Pure position control (no admittance)
    FORCE      // Force control via admittance
  };

  /**
   * @brief Constructor
   * @param nh ROS NodeHandle for subscriber
   * @param ft_topic FT sensor topic name
   */
  AdmittanceFilter(ros::NodeHandle& nh, const std::string& ft_topic = "/schunk_netbox/raw");

  ~AdmittanceFilter() = default;

  /**
   * @brief Initialize parameters from ROS parameter server
   * @param ns Namespace for parameters (e.g., "~admittance")
   * @return true if successful
   */
  bool init(const std::string& ns = "~admittance");

  /**
   * @brief Reset filter state (call when starting new trajectory)
   */
  void reset();

  /**
   * @brief Update admittance filter
   * @param dt Time step
   * @param q Current joint position (for force frame transformation)
   * @return Position offset to add to desired trajectory
   */
  Vector3d update(double dt, const cc::JointPosition& q);

  /**
   * @brief Get velocity offset from admittance
   * @return Velocity offset
   */
  Vector3d getVelocityOffset() const { return x_dot_; }
  Vector3d getAccelerationOffset() const;
  /**
   * @brief Set desired contact force
   * @param F_des Desired force vector [Fx, Fy, Fz] in Newtons
   */
  void setDesiredForce(const Vector3d& F_des) { F_desired_ = F_des; }

  /**
   * @brief Set axis control mode
   * @param axis 0=X, 1=Y, 2=Z
   * @param mode POSITION or FORCE
   */
  void setAxisMode(int axis, AxisMode mode);

  /**
   * @brief Set admittance parameters
   */
  void setMass(const Vector3d& M_d);
  void setDamping(const Vector3d& D_d);
  void setStiffness(const Vector3d& K_d);

  /**
   * @brief Get current external force (for debugging)
   */
  Vector3d getExternalForce() const;

  /**
   * @brief Check if FT sensor data is being received
   */
  bool isSensorConnected() const { return sensor_connected_; }

private:
  // FT sensor callback
  void ftSensorCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);

  // ROS
  ros::NodeHandle& nh_;
  ros::Subscriber ft_sub_;

  // Sensor data (thread-safe)
  mutable std::mutex ft_mutex_;
  Vector3d F_ext_;           // External force from sensor
  Vector3d T_ext_;           // External torque from sensor
  bool sensor_connected_;
  ros::Time last_ft_time_;
  ur::URModel model_;

  // Admittance parameters (diagonal matrices stored as vectors for simplicity)
  Vector3d M_d_;  // Virtual mass [kg]
  Vector3d D_d_;  // Virtual damping [Ns/m]
  Vector3d K_d_;  // Virtual stiffness [N/m]
  double G_; // Gravity compensation for vertical axis (if needed)
  // Desired force
  Vector3d F_desired_;

  // Axis modes
  std::array<AxisMode, 3> axis_modes_;

  // Filter state
  Vector3d x_;       // Position offset
  Vector3d x_dot_;   // Velocity offset
  Vector3d x_ddot_;  // Acceleration offset (from last update)

  // Limits for safety
  Vector3d x_max_;      // Maximum position offset
  Vector3d x_dot_max_;  // Maximum velocity
};

} // namespace tum_ics_ur_robot_lli

#endif // ADMITTANCE_FILTER_H
