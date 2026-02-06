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


#ifndef CONTROL_CORE_ROTATION3_REF_H
#define CONTROL_CORE_ROTATION3_REF_H

#include <control_core/utilities/type_guard.h>
#include <control_core/type_bases/rotation3_base.h>

namespace control_core
{
/*!
 * \brief The Rotation3Ref class.
 *
 * The Rotation3 is of type Eigen::Matrix3 and is
 * represented by the math symbol \f$\mathbf{R}\f$.
 *
 * References the data of another Eigen type class
 * via Eigen::Block.
 *
 */
template <typename _Derived>
class Rotation3Ref : 
  public Eigen::Block<_Derived, 3, 3>,
  public Rotation3Base<Rotation3Ref<_Derived> >,
  TypeGuard
{
  //OW_TYPE_GUARD(Rotation3Ref)

public:
  typedef _Derived Derived;
  typedef typename Derived::Scalar Scalar;
  typedef Eigen::Block<Derived, 3, 3> Base;
  typedef Rotation3Base<Rotation3Ref<_Derived> > RBase;

public:
  /*!
   * \brief Default Constructor.
   */
  explicit Rotation3Ref(Derived& ref, int start_row = 0, int start_col = 0) : 
    Base(ref, start_row, start_col)
  {
  }

  /*!
   * \brief Assignment operator.
   */
  Rotation3Ref operator=(const Rotation3<Scalar>& other)
  {
    Base::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment operator.
   */
  Rotation3Ref operator=(const control_core::AngularPosition<Scalar>& other)
  {
    Base::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment operator.
   */
  template <typename OtherDerived>
  Rotation3Ref operator=(const control_core::AngularPositionRef<OtherDerived>& other)
  {
    Base::operator=(other);
    return *this;
  }

  /*!
   * \brief Use assignment operators of base class.
   */
  using Base::operator=;
  using RBase::operator=;

private:
  /*!
   * \brief No null reference.
   */
  Rotation3Ref();
};

}  // namespace control_core

#endif  // CONTROL_CORE_ROTATION3_REF_H
