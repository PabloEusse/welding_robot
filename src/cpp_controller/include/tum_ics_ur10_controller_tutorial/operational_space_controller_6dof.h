#ifndef UR_ROBOT_LLI_OPERATIONAL_SPACE_CONTROLLER_6DOF_H
#define UR_ROBOT_LLI_OPERATIONAL_SPACE_CONTROLLER_6DOF_H

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include <tum_ics_ur10_controller_tutorial/AdmittanceFilter.h>
#include <ur_model/ur_model.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>
#include <memory>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>

namespace tum_ics_ur_robot_lli
{
  namespace RobotControllers
  {

    /**
     * @brief Operational Space Controller for UR10 robot
     * 
     * Implements adaptive control in both joint space and Cartesian space
     * with state machine for trajectory tracking.
     */
    class OperationalSpaceController6DOF : public ControlEffort
    {
    public:
      // State machine states
      enum class State
      {
        INIT,
        LINEAR,
        TRACK_SLOW,
        TRACK_FAST,
        END
      };

      // Controller types
      enum class ControllerType
      {
        SIMPLE_BREAK,
        PD,
        PID
      };

      // Control space
      enum class ControlSpace
      {
        JOINT,
        CARTESIAN
      };

    private:
      // First iteration flag
      bool is_first_iter_;

      // Joint states
      Vector6d q_start_;
      JointState q_init_;
      JointState q_home_;
      JointState q_park_;
      
      // ROS communication
      ros::NodeHandle nh_;
      ros::Publisher control_data_pub_;
      ros::Publisher joint_states_des_pub_;
      ros::Publisher lyapunov_pub_;
      ros::Publisher steady_state_error_pub_;
      ros::Publisher cartesian_pub_;
      ros::Publisher tf_pub_; 
      // Gain matrices (3x3 for Cartesian control)
      Matrix3d Kp_PD_;
      Matrix3d Kd_PD_;
      Matrix6d Kreg_PD_;  // 6x6 for joint space regularization
      Matrix6d Kreg_PD_cart_;

      Matrix3d Kp_PID_;
      Matrix3d Kd_PID_;
      Matrix3d Ki_PID_;

      // State machine
      State current_state_;
      ControllerType controller_type_;
      ControlSpace control_space_;


      // Time parameters
      double t_INIT_;
      double t_LINEAR_;
      double t_TRACK_SLOW_;
      double t_TRACK_FAST_;
      double t0_;          // Start time
      double last_t_;      // Time at last state transition

      // State variables
      Vector6d q_;         // Current joint position
      Vector3d X_;         // Current Cartesian position (3D)
      Vector6d last_q_;    // Joint position at last state transition
      Vector3d last_X_;    // Cartesian position at last state transition

      //adding offset
      Vector6d offset;
      JointState state_adj;
      Vector3d x_des_template;

      // Trajectory parameters
      Vector6d q0_;
      Vector6d start_track_pose_;   // Joint pose for tracking start
      Vector6d non_singular_pose_;  // Safe non-singular pose
      Vector3d orbital_center_;     // Center of circular trajectory (like Python)
      double circle_radius_;        // Radius of circular trajectory
      double vertical_amplitude_;   // Amplitude for z oscillation

      // Adaptive control parameters
      Eigen::Matrix<double, 14, 1> theta_;  // Adaptive parameters
      Eigen::Matrix<double, 14, 1> ro_;     // Adaptation gains

      // Error accumulators
      Vector3d integral_error_acc_;

      // UR10 DH parameters (stored internally for self-contained kinematics)
      // Standard UR10 parameters
      double d1_, a2_, a3_, d4_, d5_, d6_;

      ur::URModel model_;

      // Admittance filter for force control
      std::unique_ptr<AdmittanceFilter> admittance_filter_;
      bool use_admittance_;
      ros::Publisher admittance_pub_;  // Debug publisher

    public:
      /**
       * @brief Constructor
       * @param weight Controller weight (default 1.0)
       * @param name Controller name
       */
      OperationalSpaceController6DOF(double weight = 1.0, 
                                      const QString &name = "OperationalSpaceCtrl6DOF");

      ~OperationalSpaceController6DOF();

      // Setters for initial configurations
      void setQInit(const JointState &q_init);
      void setQHome(const JointState &q_home);
      void setQPark(const JointState &q_park);

    private:
      // Controller lifecycle methods (overridden from base class)
      bool init() override;
      bool start() override;
      Vector6d update(const RobotTime &time, const JointState &state) override;
      bool stop() override;

      // State machine
      void updateStateMachine(double t_elapsed);

      void publishTF_raw(const std::string& parent,
                   const std::string& child,
                   double x, double y, double z);

      // Trajectory generation
      void generateTrajectory(double t, 
                              Vector6d& q_des, Vector6d& qdot_des, Vector6d& qddot_des,
                              Vector3d& x_des, Vector3d& xdot_des, Vector3d& xddot_des);

      // Controllers
      Vector6d jointSpaceController(const JointState& state,
                                    const Vector6d& q_des,
                                    const Vector6d& qdot_des,
                                    const Vector6d& qddot_des,
                                    double dt);

      Vector6d cartesianSpaceController(const JointState& state,
                                         const Vector3d& x_current,
                                         const Vector3d& xdot_current,
                                         const Vector3d& x_des,
                                         const Vector3d& xdot_des,
                                         const Vector3d& xddot_des,
                                         double dt);

      // Helper functions
      Vector6d closestEquivalentAngles(const Vector6d& q_ref, const Vector6d& q_current);
      Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& matrix, double tolerance = 1e-6);
      double computeManipulability(const Eigen::Matrix<double, 6, 6>& J);

      // Self-contained kinematics (UR10 specific)
      Eigen::Affine3d forwardKinematics(const Vector6d& q);
      Eigen::Matrix<double, 6, 6> computeJacobian(const Vector6d& q);
      Eigen::Matrix<double, 6, 6> computeJacobianDotTranslational(const Vector6d& q, const Vector6d& qp);
      Vector6d computeYr_theta(const Vector6d& q, const Vector6d& qdot, const Vector6d& qrdot, const Vector6d& qrddot);

      Vector3d getEndEffectorPosition(const Vector6d& q);
      Vector3d getEndEffectorVelocity(const Vector6d& q, const Vector6d& qp);
    };

  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli

#endif // UR_ROBOT_LLI_OPERATIONAL_SPACE_CONTROLLER_6DOF_H
