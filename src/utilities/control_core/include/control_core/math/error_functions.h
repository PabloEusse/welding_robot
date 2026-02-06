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

#ifndef CONTROL_CORE_MATH_ERROR_FUNTIONS_H
#define CONTROL_CORE_MATH_ERROR_FUNTIONS_H

#include <control_core/math/quaternion.h>
#include <control_core/math/quaternion.h>

#include <control_core/math/error_functions.h>

// the namespace for the project
namespace cc
{

  /*!
  * \brief CartesianTransformation Error 
  * 
  * Computes the Error between a desired CartesianPosition Xd_world
  * and the current CartesianPosition Xcur_world.
  * 
  * Important:
  * The resulting error vector is expressed within the w coordinate system.
  * 
  * \return cartesian error w.r.t world
  * e_w = cartesianError(Xd_w, X_w)
  */
  inline CartesianVector& cartesianErrorWorld(
    CartesianVector& e,
    const cc::CartesianPosition& Xd,
    const cc::CartesianPosition& X)
  {
    e.linear() = Xd.linear() - X.linear();
    e.angular() = logErrorWorld(Xd.angular(), X.angular());

    return e;
  };

  /*!
  * \brief CartesianTransformation Error 
  * 
  * Computes the Error between a desired CartesianPosition Xd_world
  * and the current CartesianPosition Xcur_world.
  * 
  * Important:
  * The resulting error vector is expressed within the world coordinate system.
  * 
  * \return cartesian error w.r.t world
  * e_w = cartesianError(Xd_w, X_w)
  */
  inline CartesianVector cartesianErrorWorld(
    const cc::CartesianPosition& Xd,
    const cc::CartesianPosition& X)
  {
    CartesianVector e;
    return cartesianErrorWorld(e, Xd, X);
  };

  /*!
  * \brief CartesianTransformation Error 
  * 
  * Computes the Error between a desired HomogeneousTransformation Td_world
  * and the current HomogeneousTransformation Tcur_world.
  * 
  * Important:
  * The resulting error vector is expressed within the base coordinate system.
  * 
  * \return cartesian error w.r.t world
  * e_w = cartesianError(Td_w, T_w)
  */
  inline CartesianVector& cartesianErrorWorld(
    CartesianVector& e,
    const cc::HomogeneousTransformation& Td,
    const cc::HomogeneousTransformation& T)
  {
    control_core::AngularPosition<Scalar> Qd(Td.orien());
    control_core::AngularPosition<Scalar> Q(T.orien());
    e.linear() = Td.pos() - T.pos();
    e.angular() = logErrorWorld(Qd, Q);
    return e;
  };

  /*!
  * \brief CartesianTransformation Error 
  * 
  * Computes the Error between a desired HomogeneousTransformation Td_world
  * and the current HomogeneousTransformation Tcur_world.
  * 
  * Important:
  * The resulting error vector is expressed within the base coordinate system.
  * 
  * \return cartesian error w.r.t world
  * e_w = cartesianError(Td_w, T_w)
  */
  inline CartesianVector cartesianErrorWorld(
    const cc::HomogeneousTransformation& Td,
    const cc::HomogeneousTransformation& T)
  {
    CartesianVector e;
    return cartesianErrorWorld(e, Td, T);
  };


  /*!
   * \brief Compute the shortest connection between two Quaternions.
   * 
   * \return modified desired quaternion.
   */
  template <typename Derived1, typename Derived2>
  inline cc::AngularPosition shortestPath(
      const control_core::QuaternionBase<Derived1> &Qd,
      const control_core::QuaternionBase<Derived2> &Q)
  {
    cc::AngularPosition Qe = Qd.derived() * Q.derived().inverse();
    checkFlipQuaternionSign(Qe);
    return Qe * Q.derived();
  }

  /*!
   * \brief Compute the shortest connection between two CartesianPosition.
   * 
   * \return modified desired CartesianPosition.
   */
  inline cc::CartesianPosition shortestPath(
      const cc::CartesianPosition &Xd,
      const cc::CartesianPosition &X)
  {
    cc::CartesianPosition Xd_mod;
    Xd_mod.linear() = Xd.linear();
    Xd_mod.angular() = shortestPath(Xd.angular(), X.angular());
    return Xd_mod;
  };

  /*!
   * \brief Compute the shortest connection between two Vectors.
   * 
   * \return the desired Vector.
   */
  template <typename Derived1, typename Derived2>
  inline Eigen::Matrix<typename Derived1::Scalar, Derived1::RowsAtCompileTime, Derived1::ColsAtCompileTime>
  shortestPath(
      const Eigen::MatrixBase<Derived1> &xd,
      const Eigen::MatrixBase<Derived2> &x)
  {
    return xd;
  };

} // namespace cc

#endif