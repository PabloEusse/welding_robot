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


#ifndef CONTROL_CORE_JACOBIAN_REF_H
#define CONTROL_CORE_JACOBIAN_REF_H

#include <control_core/utilities/type_guard.h>
#include <control_core/type_bases/jacobian_base.h>

namespace cc{
/*!
 * \brief The traits class for the Jacobian class.
 *
 * This class contains the typedefs and enums for
 * the Jacobian class.
 *
 * These typedefs and enums are used to define and describe
 * the properties (traits) of this class.
 */
template <typename _Derived, int _Cols>
struct traits<control_core::JacobianRef<_Derived, _Cols> >
{
  typedef typename _Derived::Scalar Scalar;
  enum
  {
    RowsAtCompileTime = 3,
    ColsAtCompileTime = _Cols
  };
};

} // namespace cc

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
template <typename _Derived, int _Cols>
class JacobianRef :
  public Eigen::Block<_Derived, 3, _Cols>,
  public JacobianBase<JacobianRef<_Derived, _Cols> >,
  TypeGuard
{
  //OW_TYPE_GUARD(JacobianRef)

public:
  typedef _Derived Derived;
  typedef typename Derived::Scalar Scalar;
  typedef Eigen::Block<Derived, 3, _Cols> Base;
  typedef JacobianBase<JacobianRef<Derived, _Cols> > JBase;

  enum
  {
    ColsAtCompileTime = _Cols
  };

public:
  /*!
   * \brief Default Constructor.
   *
   * \param ref
   *      the reference to storage Eigen object.
   *
   * \param start_row
   *      the start index of the row for Eigen::Block.
   *
   * \param start_col
   *      the start index of the column for Eigen::Block.
   */
  explicit JacobianRef(Derived& ref, int start_row = 0, int start_col = 0) : 
    Base(ref, start_row, start_col)
  {
  }

  /*!
   * \brief Use assignment operators of base class.
   */
  using Base::operator=;
  using JBase::operator=;

private:
  /*!
   * \brief No null reference.
   */
  JacobianRef();
};

}  // namespace control_core

#endif  // CONTROL_CORE_FORCE_REF_H
