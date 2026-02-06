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


#ifndef CONTROL_CORE_QUATERNION_BASE_H
#define CONTROL_CORE_QUATERNION_BASE_H

#include <control_core/type_bases/rotation_base.h>
#include <control_core/utilities/support_templates.h>
#include <control_core/utilities/forward_declarations.h>

#include <control_core/math/maps.h>

namespace control_core
{
/*!
 * \brief The QuaternionBase class.
 */
template <typename _Derived>
class QuaternionBase : 
  public RotationBase<_Derived>
{
public:
  typedef _Derived Derived;
  typedef RotationBase<Derived> Base;

  typedef typename cc::traits<Derived>::Scalar Scalar;
  typedef typename cc::traits<Derived>::Index Index;
  enum {
    RowsAtCompileTime = cc::traits<Derived>::RowsAtCompileTime,
    ColsAtCompileTime = cc::traits<Derived>::ColsAtCompileTime,
    SizeAtCompileTime = cc::traits<Derived>::SizeAtCompileTime,
  };

public:

  /*
  * \brief get size of the Quaternion.
  */
  Index size() const 
  {
    return Base::derived().coeffs().size();
  }
  
  /*
  * \brief Direct access to Quaternion coeffs.
  */
  Scalar operator()(Index idx) const
  {
    return Base::derived().coeffs()(idx);
  }

  /*
  * \brief Direct access to Quaternion coeffs.
  */
  Scalar& operator()(Index idx)
  {
    return Base::derived().coeffs()(idx);
  }

  /*!
   * \brief Use assignment operators of base class.
   */
  using Base::operator=;

  /*!
   * \brief Conversion to euler anlges.
   */
  control_core::Vector3<Scalar> eulerAngles(Index i1, Index i2, Index i3) const
  {
    return Base::derived().toRotationMatrix().eulerAngles(i1,i2,i3);
  }

  /**
   * @brief return a quaternion that only considers yaw orientation (2d plane)
   * 
   * This function returns a Quaternion containing only the yaw orientation of
   * this.
   * 
   * @return Eigen::Quaternion<Scalar> 
   */
  Eigen::Quaternion<Scalar> yawQuaternion() const
  {
    const Scalar& w = Base::derived().w();
    const Scalar& x = Base::derived().x();
    const Scalar& y = Base::derived().y();
    const Scalar& z = Base::derived().z();

    // build quaternion around yaw axis based on yaw of original quaternion
    Eigen::Quaternion<Scalar> Q_yaw2(
      w*w + x*x - y*y - z*z, 0.0, 0.0, -2.0*x*y + 2.0*w*z);
    
    // half the yaw angle by adding I quaternion and renoramlize
    Q_yaw2.coeffs() += Eigen::Quaternion<Scalar>::Identity().coeffs();
    return Q_yaw2.normalized();
  }

  /*!
   * \brief get the element in the tangential vector space of this Lie element 
   * in cartesian vector form. 
   * 
   * The vector in tangential space is a 3d vector for a quaternions.
   * 
   * \note To calculate the error between two quaternions the signs of the 
   * coefficients need to be flipped if scalar part w < 0
   */
  control_core::Vector3<Scalar> logMap() const
  {
    return cc::logMapS3(Base::derived());
  }

  /*!
   * \brief Conversion to std::string.
   */
  std::string toString() const
  {
    std::ostringstream out;
    out << Base::derived().coeffs().transpose();
    return out.str();
  }
};

}  // namespace control_core

#endif  // CONTROL_CORE_VECTOR_REF_H
