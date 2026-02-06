#include <control_core/trajectory/polynomial_trajectories.h>
#include <control_core/trajectory/state_trajectory.h>
#include <control_core/trajectory/spline_trajectory.h>
#include <fstream>

#include <control_core/types.h>

// test cubic spline for arbitary number of points
int main()
{  
  cc::Scalar period = 1.0;

  // create a time vector
  cc::Scalar len = period/0.01;
  std::vector<cc::Scalar> time_vec;
  for(int i = 0; i < len; ++i)
    time_vec.push_back(i*0.01);

  //----------------------------------------------------------------------------
  // create a 6 order polynomial for the cartesian pose [pos, quaternion]
  // with the constrains
  cc::CartesianPosition X_start_(1, 1, 1, 1, 0, 0, 0);
  cc::CartesianPosition XP_start_(0, 0, 0, 0, 0, 0, 0);
  cc::CartesianPosition XPP_start_(0, 0, 0, 0, 0, 0, 0);
  cc::CartesianPosition X_end_(1, 1, 1, 0, 1, 0, 0);
  cc::CartesianPosition XP_end_(0, 0, 0, 0, 0, 0, 0);
  cc::CartesianPosition XPP_end_(0, 0, 0, 0, 0, 0, 0);
  cc::CartesianPosition X_middle_(6, 6, 6, 0.37845, 0.3052338, 0.6179029, 0.6179029);

  // create State Trajectory with type CartesianState based on a 6 Ord. Polynomial
  control_core::StateTrajectory<cc::CartesianState> 
    state_traj(control_core::Polynomial6Order(
        period, X_start_, XP_start_, XPP_start_,
        X_end_, XP_end_, XPP_end_, X_middle_));

  // interpolate the complete state
  // note: evaluate can also take a single time t
  std::vector<cc::CartesianState> c_results = state_traj.evaluate(time_vec);

  // save somewhere
  std::ofstream myfile;
  myfile.open("/home/simon/ros/workspaces/tmp/tests/polynomial_X.txt");
  for(int i = 0; i < time_vec.size(); ++i)
  {
    myfile << time_vec[i] << " "
      << c_results[i].pos().toString() << " "         /* position */
      << c_results[i].vel().toString() << " "         /* velocity */
      << c_results[i].acc().toString() << std::endl;  /* accleration */
  }
  myfile.close();

  // ---------------------------------------------------------------------------
  // create a Quaternion 3 order spline
  cc::AngularPosition Q_start_(1, 0, 0, 0);
  cc::AngularPosition QP_start_(0, 0, 0, 0);
  cc::AngularPosition Q_end_(0, 0, 0, 1);
  cc::AngularPosition QP_end_(0, 0, 0, 0);

  control_core::StateTrajectory<cc::AngularState> 
    Q_state_traj(control_core::Polynomial3Order(
        period, Q_start_, QP_start_,
        Q_end_, QP_end_));

  // interpolate the complete state
  // note: evaluate can also take a single time t
  std::vector<cc::AngularState> q_results = Q_state_traj.evaluate(time_vec);

  // save somewhere
  myfile.open("/home/simon/ros/workspaces/tmp/tests/polynomial_Q.txt");
  for(int i = 0; i < time_vec.size(); ++i)
  {
    myfile << time_vec[i] << " "
      << q_results[i].pos().toString() << " "        
      << q_results[i].vel().toString() << " "      
      << q_results[i].acc().toString() << std::endl;  
  }
  myfile.close();
}