#include <control_core/trajectory/polynomial_trajectories.h>
#include <control_core/trajectory/state_trajectory.h>
#include <control_core/trajectory/spline_trajectory.h>
#include <fstream>

#include <control_core/types.h>

// test cubic spline for arbitary number of points
int main()
{  
  //----------------------------------------------------------------------------
  // note: I add a extra support point in the first and last interfal in order to
  // create a spline with zero inital acceleration (needed new constrains...)

  std::vector<cc::Scalar> segments = {0, 1, 3, 5, 6, 9, 10};  
  std::vector<cc::LinearPosition> samples = {
    cc::LinearPosition(1, 1, 1),
    cc::LinearPosition(0, 0, 0),  // support point
    cc::LinearPosition(5, 5, 5),
    cc::LinearPosition(0, 0, 0),
    cc::LinearPosition(2, 2, 2),
    cc::LinearPosition(0, 0, 0),  // support point
    cc::LinearPosition(3, 3, 3)
  };

  for(unsigned int i = 0; i < segments.size() - 1; ++i)
    std::cout << "idx=" << i << " start=" << segments[i] << " end="
              << segments[i+1] << std::endl;

  control_core::PolynomialTrajectory<cc::LinearPosition, cc::Scalar>
    spline = control_core::CubicSpline(segments, samples);

  std::cout << "start_time=" << spline.startTime() << std::endl;
  std::cout << "end_time=" << spline.endTime() << std::endl;

  double step = 0.01;
  int len = (spline.endTime() - spline.startTime())/step;
  std::vector<double> time_vec;
  for(int i = 0; i < len; ++i)
    time_vec.push_back(step*i);

  std::vector<cc::LinearPosition> resutls_vec = spline.evaluate(time_vec);

  std::vector<std::vector<cc::LinearPosition> > resutls_vec_all = 
    spline.evaluateAll(time_vec);

  std::ofstream myfile;
  myfile.open("/home/simon/ros/workspaces/tmp/tests/spline_curve.txt");
  for(int i = 0; i < time_vec.size(); ++i)
  {
    myfile << time_vec[i] << " "; 
    for(int j = 0; j < resutls_vec_all[i].size(); ++j) 
    {
      myfile << resutls_vec_all[i][j].toString() << " ";
    }
    myfile << std::endl;
  }
  myfile.close();

  //----------------------------------------------------------------------------
  // same test but this time with our statebase class
 
  // create cartesian pose samples
  std::vector<cc::AngularPosition> a_samples = {
    cc::AngularPosition(1, 0, 0, 0),
    cc::AngularPosition::Zero(),  // support point
    cc::AngularPosition(0.7042004, 0, 0.7100013, 0),
    cc::AngularPosition(0.4388107, 0, 0.6353917, 0.6353917),
    cc::AngularPosition(0.37845, 0.3052338, 0.6179029, 0.6179029),
    cc::AngularPosition::Zero(),  // support point
    cc::AngularPosition(1, 0, 0, 0)
  };
  
  // feed into interpolator
  control_core::StateTrajectory<cc::AngularState> 
    state_traj(control_core::CubicSpline(segments, a_samples));

  // interpolate the state
  std::vector<cc::AngularState> c_results = state_traj.evaluate(time_vec);
  return 0;
}