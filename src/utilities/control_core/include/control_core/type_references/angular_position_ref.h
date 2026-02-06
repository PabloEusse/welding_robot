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


#ifndef CONTROL_CORE_ANGULAR_POSITION_REF_H
#define CONTROL_CORE_ANGULAR_POSITION_REF_H

#include <control_core/utilities/type_guard.h>
#include <control_core/eigen/quaternion_ref.h>
#include <control_core/type_bases/angular_position_base.h>

namespace cc{

/*!
 * \brief The traits class for the AngularPosition class.
 *
 * This class contains the typedefs and enums for
 * the AngularPosition class.
 *
 * These typedefs and enums are used to define and describe
 * the properties (traits) of this class.
 */
template<typename _Derived>
struct traits<control_core::AngularPositionRef<_Derived> >
{
  typedef typename _Derived::Scalar Scalar;
  typedef int Index;

  enum {
    RowsAtCompileTime = 4,
    ColsAtCompileTime = 1,
    SizeAtCompileTime = 4,
  };
};

} // namespace cc

namespace control_core
{
/*!
 * \brief The AngularPositionRef class.
 *
 * The AngularPositionRef is of type Eigen::QuaternionRef and is
 * represented by the math symbol \f$\mathbf{Q}\f$.
 *
 * References the data of another Eigen type class
 * via Eigen:Block.
 *
 */
template <typename _Derived>
class AngularPositionRef : 
  public Eigen::QuaternionRef<_Derived>,
  public AngularPositionBase<AngularPositionRef<_Derived> >,
  TypeGuard
{
  //OW_TYPE_GUARD(AngularPositionRef)

public:
  typedef _Derived Derived;
  typedef typename Derived::Scalar Scalar;
  typedef Eigen::QuaternionRef<Derived> Base;
  typedef AngularPositionBase<AngularPositionRef<Derived> > ABase;


public:
  /*!
   * \brief Default Constructor.
   *
   * \param ref
   *      the r eference to storage Eigen object to access the
   *      elements of the quaternion via Eigen::Block.
   *
   * \param start_row
   *      the start index of the row for Eigen::Block.
   *
   * \param start_col
   *      the start index of the column for Eigen::Block.
   */
  explicit AngularPositionRef(Derived& ref, int start_row = 0, int start_col = 0) : 
    Base(ref, start_row, start_col)
  {
  }

  /*!
   * \brief Use assignment operators of base class.
   */
  using Base::operator=;
  using ABase::operator=;

  /*!
   * \brief Assignment operator AngularPosition.
   */
  AngularPositionRef operator=(const AngularPosition<Scalar>& other)
  {
    Base::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment of other AngularPositionRef.
   *
   */
  template <typename _OtherDerived>
  AngularPositionRef& operator=(const AngularPositionRef<_OtherDerived>& other)
  {
    Base::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment operator Rotation3.
   */
  AngularPositionRef operator=(const Rotation3<Scalar>& other)
  {
    Base::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment operator Rotation3Ref.
   */
  template <typename OtherDerived>
  AngularPositionRef operator=(const Rotation3Ref<OtherDerived>& other)
  {
    Base::operator=(other);
    return *this;
  }

private:
  /*!
   * \brief No null reference.
   */
  AngularPositionRef();
};

}  // namespace control_core

#endif  // CONTROL_CORE_ANGULAR_POSITION_REF_H
