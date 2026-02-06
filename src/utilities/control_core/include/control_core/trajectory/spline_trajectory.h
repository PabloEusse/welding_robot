  /*! \file
 *
 * \author Simon Armleder
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */

#ifndef CONTROL_CORE_SPLINE_TRAJECTORIES_H
#define CONTROL_CORE_SPLINE_TRAJECTORIES_H

#include <control_core/trajectory/polynomial_trajectory.h>
#include <control_core/math/error_functions.h>

namespace control_core{
  
  /*!
   * \brief Builds a cublic spline curve with arbitrary many sample points.
   * 
   * The minimum number of points is 5 (3 for cublic spline + 2 to match
   * the initial zero velocity + inital zero acceleration constrain).
   * 
   * The two extra points are inserted at the FIRST and FINAL segment and have 
   * arbitary values. (only their position in time matters).
   * E.g.:
   *  segments=[0, 0.3, 1, 1.7, 2] // time
   *  samples =[5, NAN, 9, NAN, 5] // value (NAN is arbitary)
   * 
   * Gives a spline through three points (t=0s, 5), (t=1s, 9), (t=2s, 5)
   * The inital and final velocity and acceleration are zero.
   * 
   * Usage:
   *  std::vector<cc::Scalar> segments = {0, 1, 3, 5, 6, 9, 10};  
   *  std::vector<cc::LinearPosition> samples = {
   *    cc::LinearPosition(1, 1, 1),
   *    cc::LinearPosition(-1, -1, -1),  // support point
   *    cc::LinearPosition(5, 5, 5),
   *    cc::LinearPosition(0, 0, 0),
   *    cc::LinearPosition(2, 2, 2),
   *    cc::LinearPosition(-1, -1, -1),  // support point
   *    cc::LinearPosition(3, 3, 3)
   *   };
   * 
   *  control_core::PolynomialTrajectory<cc::LinearPosition, cc::Scalar>
   *    spline = control_core::CubicSpline(segments, samples);
   * 
   *  std::vector<cc::LinearPosition> resutls_vec = spline.evaluate(time_vec);
   */
  template<typename _Value>
  static PolynomialTrajectory<_Value, typename _Value::Scalar> CubicSpline(
    const std::vector<typename _Value::Scalar>& segments,
    const std::vector<_Value>& samples)
  {
    typedef _Value Value;
    typedef typename Value::Scalar Scalar;
    typedef typename PolynomialTrajectory<Value,Scalar>::PolynomialMatrix PolynomialMatrix;
    typedef typename PolynomialTrajectory<Value,Scalar>:: PolynomialType PolynomialType;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorX;

    int n_dim = samples[0].size();
    int n = segments.size();    
    int m = segments.size()-1;

    MatrixX As = MatrixX::Zero(n,n);
    VectorX bs = VectorX::Zero(n);
    VectorX h = VectorX::Zero(m);

    // compute the segment length
    for(int i = 0; i < m; ++i)
    {
      h[i] = segments[i+1] - segments[i];
    }

    // compute the shortest distance between samples
    std::vector<Value> samples_(n);
    samples_[0] = samples[0];
    for(int i = 1; i < n; ++i)
    {
      samples_[i] = cc::shortestPath(samples[i], samples_[i-1]);
    }

    // build the polynomials for each segment
    std::vector<Scalar> a(n);
    PolynomialMatrix polynomials(n-1);

    for(int j = 0; j < n_dim; ++j)
    {
      // assample all samples of the same dimention into a vector
      for(int i = 0; i < n; ++i) 
        a[i] = samples[i](j);

      // build linear system
      As.setZero();
      bs.setZero();

      // zero acc at begining
      As(0,0) = 1;

      // modified segment s0 between x1 and r1 (substitude a1)
      // a1 = a0 + c1/3*h_0^2
      As(1,0) = h[0];
      As(1,1) = 3*h[0] + 2*h[1] + h[0]*h[0]/h[1];
      As(1,2) = h[1];
      bs[1] = 3/h[1]*(a[2]-a[0]);

      if(n == 5)
      {
        // only ONE point (x3) in the center.
        // Therefore the left and right modified segments overlapp
        As(2,1) = h[1]-h[0]*h[0]/h[1];
        As(2,2) = 2*(h[2] + h[1]);
        As(2,3) = h[2]-h[3]*h[3]/h[2];
        bs[2] = -(3/h[1])*(a[2]-a[0]) + 3/h[2]*(a[4] - a[2]);
      }
      else
      {
        // MULTIPLE points
        // Therefore the left and right modified segments don't overlapp
        As(2,1) = h[1]-h[0]*h[0]/h[1];
        As(2,2) = 2*(h[2] + h[1]);
        As(2,3) = h[2];
        bs[2] = -(3/h[1])*(a[2]-a[0]) + 3/h[2]*(a[3] - a[2]);

        // build matrix in between (standard spline function)
        for(int i = 3; i < n-3; ++i)
        {
          As(i,i-1) = h[i-1];
          As(i,i) = 2*(h[i]+h[i-1]);
          As(i,i+1) = h[i];
          bs[i] = (3/h[i])*(a[i+1]-a[i]) - (3/h[i-1])*(a[i]-a[i-1]);
        }

        // modified segment s[n-2] (substitude a[n-1])
        As(n-3,n-4) = h[m-3];
        As(n-3,n-3) = 2*(h[m-3] + h[m-2]);
        As(n-3,n-2) = h[m-2]-h[m-1]*h[m-1]/h[m-2];
        bs[n-3] = -(3/h[m-3])*(a[n-3]-a[n-4]) + (3/h[m-2])*(a[n-1] - a[n-3]);
      }

      // modified segment s[n-1] (substitude a[n-1])
      As(n-2,n-3) = h[m-2];
      As(n-2,n-2) = 3*h[m-1] + 2*h[m-2] + h[m-1]*h[m-1]/h[m-2];
      As(n-2,n-1) = h[m-1];
      bs[n-2] = -3/h[m-2]*(a[n-1]-a[n-3]);

      // zero acceleration in the end
      As(n-1,n-1) = 1;

      // solve the linear system
      VectorX c = As.householderQr().solve(bs);
      // now a1 can be computed: c1/3*h0**2 + a0 == a1
      a[1] = a[0] + h[0]*h[0]*(c[1]/3);
      // now a can be computed: c3*h4**2/3 + a[n] == a[n-1]
      a[n-2] = a[n-1] + h[m-1]*h[m-1]*(c[n-2]/3);
      
      // build the polynomials for each segment
      VectorX coeff(4);
      for(int i = 0; i < n-1; ++i)
      {
        coeff[0] = a[i];
        coeff[1] = (1/h[i])*(a[i+1]-a[i])-(h[i]/3)*(2*c[i]+c[i+1]);
        coeff[2] = c[i];
        coeff[3] = (1/(3*h[i]))*(c[i+1]-c[i]);
        polynomials[i].push_back(PolynomialType(coeff, segments[i]));
      }
    }
    std::vector<Scalar> segments_(segments.begin(), segments.end()-1);
    return PolynomialTrajectory<_Value,Scalar>(segments_, polynomials);
  }

}

#endif