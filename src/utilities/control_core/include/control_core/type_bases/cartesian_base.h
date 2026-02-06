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


#ifndef CONTROL_CORE_CARTESIAN_BASE_H
#define CONTROL_CORE_CARTESIAN_BASE_H

#include <Eigen/Dense>
#include <control_core/utilities/forward_declarations.h>
#include <control_core/type_bases/vector_base.h>

namespace control_core
{
/*!
 * \brief The CartesianBase class.
 */
template <typename _Derived>
class CartesianBase : 
  public VectorBase<_Derived>
{
public:
  typedef _Derived Derived;
  typedef VectorBase<_Derived> Base;

  typedef typename cc::traits<Derived>::Base::Scalar Scalar;
  typedef typename cc::traits<Derived>::LinearRef LinearRef;
  typedef typename cc::traits<Derived>::AngularRef AngularRef;
  typedef typename cc::traits<Derived>::CLinearRef CLinearRef;
  typedef typename cc::traits<Derived>::CAngularRef CAngularRef;
  enum {
    LinearPartAtCompileTime = cc::traits<Derived>::LinearPartAtCompileTime,
    AngularPartCompileTime = cc::traits<Derived>::AngularPartCompileTime,
  };

public:
  /*!
   * \brief access to linear part
   */
  LinearRef linear()
  {
    return LinearRef(Base::derived(), LinearPartAtCompileTime);
  }

  /*!
   * \brief const access to linear part
   */
  CLinearRef linear() const
  {
    return CLinearRef(Base::derived(), LinearPartAtCompileTime);
  }

  /*!
   * \brief access to angular part
   */
  AngularRef angular()
  {
    return AngularRef(Base::derived(), AngularPartCompileTime);
  }

  /*!
   * \brief const access to angular part
   */
  CAngularRef angular() const
  {
    return CAngularRef(Base::derived(), AngularPartCompileTime);
  }

  /*!
   * \brief Assignment operator
   */
  template<typename OtherDerived>
  CartesianBase& operator=(const CartesianBase<OtherDerived>& other)
  {
    linear() = static_cast<const control_core::Vector3<Scalar>& >(other.linear());
    angular() = static_cast<const control_core::Vector3<Scalar>& >(other.angular());
    return *this;
  }
};

}  // namespace control_core

#endif  // CONTROL_CORE_CARTESIAN_BASE_H
