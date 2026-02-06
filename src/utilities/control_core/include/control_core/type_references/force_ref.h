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


#ifndef CONTROL_CORE_FORCE_REF_H
#define CONTROL_CORE_FORCE_REF_H

#include <control_core/utilities/type_guard.h>
#include <control_core/type_bases/force_base.h>

namespace control_core{

/*!
 * \brief The ForceRef class.
 *
 * The ForceRef is of type Eigen::Vector3 and is
 * represented by the math symbol \f$\mathbf{f}\f$.
 *
 * References the data of another Eigen type class
 * via Eigen::Block.
 *
 */
template <typename _Derived>
class ForceRef :
  public Eigen::Block<_Derived, 3, 1>,
  public ForceBase<ForceRef<_Derived> >,
  TypeGuard
{
  //OW_TYPE_GUARD(ForceRef)

public:
  typedef _Derived Derived;
  typedef typename Derived::Scalar Scalar;
  typedef Eigen::Block<Derived, 3, 1> Base;
  typedef ForceBase<ForceRef<Derived> > FBase;

public:
  /*!
   * \brief Default Constructor.
   *
   * \param ref
   *      the reference to storage Eigen object to access the
   *      elements of the LinearVelocityn via Eigen::Block.
   *
   * \param start_row
   *      the start index of the row for Eigen::Block.
   *
   * \param start_col
   *      the start index of the column for Eigen::Block.
   */
  explicit ForceRef(Derived& ref, int start_row = 0, int start_col = 0) : 
    Base(ref, start_row, start_col)
  {
  }

  /*!
   * \brief Use assignment operators of base class.
   */
  using Base::operator=;
  using FBase::operator=;

  /*!
   * \brief Assignment operator.
   */
  ForceRef operator=(const Force<Scalar>& other)
  {
    Base::operator=(other);
    return *this;
  }

private:
  /*!
   * \brief No null reference.
   */
  ForceRef();
};

}  // namespace control_core

#endif  // CONTROL_CORE_FORCE_REF_H
