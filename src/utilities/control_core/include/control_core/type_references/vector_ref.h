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


#ifndef CONTROL_CORE_VECTOR_REF_H
#define CONTROL_CORE_VECTOR_REF_H

#include <control_core/utilities/type_guard.h>
#include <control_core/type_bases/vector_base.h>

namespace control_core
{
/*!
 * \brief The VectorRef class.
 *
 * References the data of another Eigen type class
 * via Eigen:Block.
 *
 * We need this special type to get the behavior of Eigen::Matrix
 * when defining new references.
 */
template <typename _Derived, int _Rows = Eigen::Dynamic>
class VectorRef : 
  public Eigen::Block<_Derived, _Rows, 1>,
  public VectorBase<VectorRef<_Derived, _Rows> >,
  TypeGuard
{
  //OW_TYPE_GUARD(VectorRef)

public:
  typedef _Derived Derived;
  typedef typename Derived::Scalar Scalar;

  enum
  {
    RowsAtCompileTime = _Rows
  };

  typedef Eigen::Block<Derived, RowsAtCompileTime, 1> Base;
  typedef VectorBase<VectorRef<Derived, RowsAtCompileTime> > VBase;

public:
  /*!
   * \brief Default Constructor, Fixed Sized Vector.
   *
   * \param ref
   *      the reference to storage Eigen object to access the
   *      elements of the quaternion via Eigen::Block.
   *
   * \param start_row
   *      the start index of the row for Eigen::Block.
   *
   * \param start_col
   *      the start index of the column for Eigen::Block.
   */
  explicit VectorRef(Derived& ref, 
                     int start_row = 0, 
                     int start_col = 0) : 
    Base(ref, start_row, start_col)
  {
  }

  /*!
   * \brief Default Constructor, Dynamically Sized Vector.
   *
   * \param ref
   *      the reference to storage Eigen object to access the
   *      elements of the quaternion via Eigen::Block.
   *
   * \param start_row
   *      the start index of the row for Eigen::Block.
   *
   * \param start_col
   *      the start index of the column for Eigen::Block.
   * 
   * \param block_rows
   *      the start index of the column for Eigen::Block.
   */
  VectorRef(Derived& ref, 
            int start_row, 
            int start_col,
            int block_rows) : 
    Base(ref, start_row, start_col, block_rows, 1)
  {
  }

  /*!
   * \brief Use assignment operators of base class.
   */
  using Base::operator=;
  using VBase::operator=;

private:
  /*!
   * \brief No null reference.
   */
  VectorRef();
};

}  // namespace control_core

#endif  // CONTROL_CORE_VECTOR_REF_H
