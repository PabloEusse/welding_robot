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


#ifndef CONTROL_CORE_TRAJECTORIES_H
#define CONTROL_CORE_TRAJECTORIES_H

#include <control_core/trajectory/polynomial_trajectory.h>
#include <control_core/math/error_functions.h>

namespace control_core{

  /*!
   * \brief Builds a third order polynomial trajectory for a vector/matrix.
   * 
   * The duration is given by period, the inital and final 
   * position and velocity by x_start, xP_start, x_end, xP_end.
   * 
   * Usage:
   * 
   *  control_core::PolynomialTrajectory<cc::LinearPosition, cc::Scalar> pos_poly3 = 
   *    control_core::Polynomial3Order(
   *      period, x_start_, xP_start_, x_end_, xP_end_);
   */
  template<typename _Value>
  static PolynomialTrajectory<_Value, typename _Value::Scalar> Polynomial3Order(
    typename _Value::Scalar period,
    const _Value& x_start,
    const _Value& xP_start,
    const _Value& x_end,
    const _Value& xP_end)
  {
    typedef _Value Value;
    typedef typename _Value::Scalar Scalar;
    typedef typename PolynomialTrajectory<Value,Scalar>::PolynomialVec PolynomialVec;
    typedef typename PolynomialTrajectory<Value,Scalar>::PolynomialType PolynomialType;

    // check that shortest path is interpolated
    Value x_end_ = cc::shortestPath(x_end, x_start);

    // build the polynomials
    PolynomialVec polynomials;
    for(int i = 0; i < x_start.size(); ++i)
    {
      polynomials.push_back(PolynomialType::Polynomial3Order(
        period, x_start(i), xP_start(i), x_end_(i), xP_end(i)));
    }
    return PolynomialTrajectory<_Value,Scalar>(polynomials);
  };

  /*!
   * \brief Builds a fifth order polynomial trajectory for a vector/matrix.
   * 
   * The duration is given by period, the inital and final 
   * position, velocity and acceleration by x_start, xP_start, xPP_start, 
   * x_end, xP_end, xPP_end.
   * 
   * Usage:
   * 
   *  control_core::PolynomialTrajectory<cc::LinearPosition, cc::Scalar> pos_poly5 = 
   *    control_core::Polynomial5Order(
   *      period, x_start, xP_start, xPP_start, x_end, xP_end, xPP_end);
   */
  template<typename _Value>
  static PolynomialTrajectory<_Value, typename _Value::Scalar> Polynomial5Order(
    typename _Value::Scalar period,
    _Value x_start,
    _Value xP_start,
    _Value xPP_start,
    _Value x_end,
    _Value xP_end,
    _Value xPP_end)
  {
    typedef _Value Value;
    typedef typename Value::Scalar Scalar;
    typedef typename PolynomialTrajectory<Value,Scalar>::PolynomialVec PolynomialVec;
    typedef typename PolynomialTrajectory<Value,Scalar>::PolynomialType PolynomialType;

    // check that shortest path is interpolated
    Value x_end_ = cc::shortestPath(x_end, x_start);

    // build the polynomials
    PolynomialVec polynomials;
    for(int i = 0; i < x_start.size(); ++i)
    {
      polynomials.push_back(PolynomialType::Polynomial5Order(
        period, x_start(i), xP_start(i), xPP_start(i), 
        x_end(i), xP_end(i), xPP_end(i)));
    }
    return PolynomialTrajectory<_Value,Scalar>(polynomials);
  };

  /*!
   * \brief Builds a six order matrix polynomial.
   * 
   * The duration is given by period, the inital and final 
   * position, velocity and acceleration by x_start, xP_start, xPP_start, 
   * x_end, xP_end, xPP_end. Additionally the middle position is definde by
   * x_middle.
   *
   * Usage:
   * 
   *  control_core::PolynomialTrajectory<cc::LinearPosition, cc::Scalar> pos_poly6 = 
   *    control_core::Polynomial6Order(
   *      period, x_start, xP_start, xPP_start, x_end, xP_end, xPP_end, x_middle);
   */
  template<typename _Value>
  static PolynomialTrajectory<_Value, typename _Value::Scalar> Polynomial6Order(
    typename _Value::Scalar period,
    const _Value& x_start,
    const _Value& xP_start,
    const _Value& xPP_start,
    const _Value& x_end,
    const _Value& xP_end,
    const _Value& xPP_end,
    const _Value& x_middle,
    typename _Value::Scalar s = 0.5)
  {
    typedef _Value Value;
    typedef typename Value::Scalar Scalar;
    typedef typename PolynomialTrajectory<Value,Scalar>::PolynomialVec PolynomialVec;
    typedef typename PolynomialTrajectory<Value,Scalar>::PolynomialType PolynomialType;

    // check that shortest path is interpolated
    Value x_middle_ = cc::shortestPath(x_middle, x_start);
    Value x_end_ = cc::shortestPath(x_end, x_middle_);

    // build the polynomials
    PolynomialVec polynomials;
    for(int i = 0; i < x_start.size(); ++i)
    {
      polynomials.push_back(PolynomialType::Polynomial6Order(
        period, x_start(i), xP_start(i), xPP_start(i), 
        x_end_(i), xP_end(i), xPP_end(i), x_middle_(i), s));
    }
    return PolynomialTrajectory<_Value,Scalar>(polynomials);
  };

} // namespace control_core

#endif // CONTROL_CORE_I_TRAJECTORIES_H
