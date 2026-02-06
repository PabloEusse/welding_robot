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

#ifndef CONTROL_CORE_MATH_H
#define CONTROL_CORE_MATH_H

/*! \file math.h
 *  \brief Contains global math functions and operations.
 */

#include <math.h>

#include <control_core/math/pseudo_inverse.h>
#include <control_core/math/transform.h>
#include <control_core/math/quaternion.h>
#include <control_core/math/error_functions.h>
#include <control_core/math/cross_product.h>

// the namespace for the project
namespace cc
{

  /*!
  * \brief Compute the factorial of x.
  *
  * \returns \f$\mathScalar{x}!\f$
  */
  inline Scalar factorial(Scalar x)
  {
    return tgamma(x + 1.0);
  }

  /*!
  * \brief Get the sign of x.
  *
  * \returns sign(x)
  */
  inline int sign(Scalar x)
  {
    return (Scalar(0) < x) - (x < Scalar(0));
  }

  /*!
  * \brief Convert degree to radian
  *
  * \returns rad
  */
  inline Scalar deg2rad(Scalar deg)
  {
    static const Scalar fac = M_PI / Scalar(180.0);
    return fac * deg;
  }

  /*!
  * \brief Convert radian to degree
  *
  * \returns rad
  */
  inline Scalar rad2deg(Scalar rad)
  {
    static const Scalar fac = Scalar(180.0) / M_PI;
    return fac * rad;
  }

  template <typename Derived>
  void rad2deg(
      Eigen::MatrixBase<Derived> &val,
      typename Eigen::MatrixBase<Derived>::Scalar lower,
      typename Eigen::MatrixBase<Derived>::Scalar upper)
  {
    val = val.cwiseMin(upper).cwiseMax(lower);
  }

  /*!
  * \brief Clamp Matrix val into given range [lower, upper].
  */
  template <typename Derived1, typename Derived2, typename Derived3>
  void clamp(
      Eigen::MatrixBase<Derived1> &val,
      const Eigen::MatrixBase<Derived2> &lower,
      const Eigen::MatrixBase<Derived3> &upper)
  {
    val = val.cwiseMin(upper).cwiseMax(lower);
  }

  /*!
  * \brief Clamp Matrix val into given range [lower, upper] and set valP to zero.
  * 
  * Sets elements in the velocity vector valP to zero if bounds are exceeded.
  * 
  * \returns true if clamping was applied
  */
  template <typename Derived1, typename Derived2, typename Derived3, typename Derived4>
  bool clampSetZero(
      Eigen::MatrixBase<Derived1> &val,
      Eigen::MatrixBase<Derived2> &valP,
      const Eigen::MatrixBase<Derived3> &lower,
      const Eigen::MatrixBase<Derived4> &upper)
  {
    bool clamp = false;
    for (size_t i = 0; i < val.size(); ++i)
    {
      if (val(i) > upper(i))
      {
        clamp = true;
        val(i) = upper(i);
        valP(i) = 0.0;
      }
      else if (val(i) < lower(i))
      {
        clamp = true;
        val(i) = lower(i);
        valP(i) = 0.0;
      }
    }
    return true;
  }

  template <typename Derived1, typename Derived2>
  bool clampSetZero(
      control_core::LinearPositionRef<Derived1> val,
      control_core::LinearVelocityRef<Derived2> valP,
      const cc::LinearPosition &lower,
      const cc::LinearPosition &upper)
  {
    bool clamp = false;
    for (size_t i = 0; i < val.size(); ++i)
    {
      if (val(i) > upper(i))
      {
        clamp = true;
        val(i) = upper(i);
        valP(i) = 0.0;
      }
      else if (val(i) < lower(i))
      {
        clamp = true;
        val(i) = lower(i);
        valP(i) = 0.0;
      }
    }
    return true;
  }

} // namespace cc

#endif // CONTROL_CORE_MATH_H
