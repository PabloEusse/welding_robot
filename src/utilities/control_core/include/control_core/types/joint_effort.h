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


#ifndef CONTROL_CORE_JOINT_EFFORT_H
#define CONTROL_CORE_JOINT_EFFORT_H

#include <Eigen/Dense>
#include <control_core/utilities/type_guard.h>
#include <control_core/type_bases/vector_base.h>
#include <control_core/type_references/joint_ref.h>

namespace control_core
{
/*!
 * \brief The JointEffort class.
 *
 * The JointEffort is of type VectorDof and is
 * represented by the math symbol \f$\mathbf{\tau}\f$.
 *
 */
template <typename _Scalar, int _Rows = Eigen::Dynamic>
class JointEffort :
  public Eigen::Matrix<_Scalar, _Rows, 1>,
  public VectorBase<JointEffort<_Scalar, _Rows> >,
  TypeGuard
{
  //OW_TYPE_GUARD(JointEffort)

public:
  typedef _Scalar Scalar;

  enum
  {
      RowsAtCompileTime = _Rows,
  };

  typedef Eigen::Matrix<_Scalar, _Rows, 1> Base;
  typedef VectorBase<JointEffort<_Scalar, _Rows> > VBase;

public:
  /*!
   * \brief Default Constructor.
   */
  JointEffort()
  {
  }

  /*!
   * \brief This constructor is for both 1x1 matrices and dynamic vectors
   */
  template<typename T>
  explicit JointEffort(const T& x) :
    Base(x)
  {
  }

  /*!
   * \brief Copy constructor.
   */
  JointEffort(const JointEffort& other) :
    Base(other)
  {
  }

  /*!
   *  \brief Copy constructor.
   *
   * This copy constructor not only works with Eigen matrices
   * but also with their expressions.
   */
  template<typename OtherDerived>
  JointEffort(const Eigen::EigenBase<OtherDerived>& other) :
    Base(other)
  {
  }

  /*!
   * \brief Copy constructor Reference.
   *
   * This copy constructor allows construction from
   * the reference class.
   */
  template <typename OtherDerived>
  JointEffort(const JointRef<OtherDerived>& other) :
    Base(other)
  {
  }

  /*!
   * \brief Use assignment operators of base class.
   */
  using Base::operator=;
  using VBase::operator=;

  /*!
   * \brief Assignment operator.
   */
  JointEffort& operator=(const JointEffort& other)
  {
    Base::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment operator or reference.
   */
  template<typename OtherDerived>
  JointEffort& operator=(const JointRef<OtherDerived>& other)
  {
    Base::operator=(other);
    return *this;
  }
  
  /*!
   * \brief Get a reference to sub vector
   */
  JointRef<Base> ref(
    int start_row,
    int block_rows) 
  {
    return JointRef<Base>(*this, start_row, block_rows);
  }

  /*!
   * \brief Get a reference to sub vector
   */
  JointRef<const Base> ref(
    int start_row,
    int block_rows) const
  {
    return JointRef<const Base>(*this, start_row, block_rows);
  }
  
};

}  // namespace control_core


#endif  // CONTROL_CORE_JOINT_EFFORT_H
