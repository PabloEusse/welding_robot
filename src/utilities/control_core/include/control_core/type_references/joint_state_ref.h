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


#ifndef CONTROL_CORE_JOINT_STATE_REF_H
#define CONTROL_CORE_JOINT_STATE_REF_H

#include <control_core/utilities/type_guard.h>
#include <control_core/type_bases/state_base.h>
#include <control_core/type_references/joint_ref.h>

namespace cc{

/*!
 * \brief The traits class for the AngularState class.
 *
 * This class contains the typedefs and enums for
 * the AngularState class.
 *
 * These typedefs and enums are used to define and describe
 * the properties (traits) of this class.
 */
template<typename _Derived>
struct traits<control_core::JointStateRef<_Derived> >
{
  typedef typename _Derived::Scalar Scalar;
  typedef control_core::JointRef<typename cc::traits<_Derived>::Pos::Base> Pos;
  typedef control_core::JointRef<typename cc::traits<_Derived>::Vel::Base> Vel;
  typedef control_core::JointRef<typename cc::traits<_Derived>::Acc::Base> Acc;
  typedef control_core::JointRef<typename cc::traits<_Derived>::Effort::Base> Effort;
  typedef control_core::JointRef<const typename cc::traits<_Derived>::Pos::Base> CPos;
  typedef control_core::JointRef<const typename cc::traits<_Derived>::Vel::Base> CVel;
  typedef control_core::JointRef<const typename cc::traits<_Derived>::Acc::Base> CAcc;
  typedef control_core::JointRef<const typename cc::traits<_Derived>::Effort::Base> CEffort;
  enum
  {
    IsRef = 1,
  };
};

} // namespace cc

namespace control_core
{
/*!
 * \brief The JointStateRef class.
 *
 * References the data of another Eigen type class
 * via Eigen:Block.
 *
 * We need this special type to get the behavior of Eigen::Matrix
 * when defining new references.
 */
template <typename _Derived>
class JointStateRef : 
  public StateBase<JointStateRef<_Derived> >, 
  TypeGuard
{
  //OW_TYPE_GUARD(JointStateRef)

public:

  typedef _Derived Derived;

  typedef control_core::JointRef<typename cc::traits<Derived>::Pos::Base> Pos;
  typedef control_core::JointRef<typename cc::traits<Derived>::Vel::Base> Vel;
  typedef control_core::JointRef<typename cc::traits<Derived>::Acc::Base> Acc;
  typedef control_core::JointRef<typename cc::traits<Derived>::Effort::Base> Effort;

  typedef control_core::JointRef<const typename cc::traits<Derived>::Pos::Base> CPos;
  typedef control_core::JointRef<const typename cc::traits<Derived>::Vel::Base> CVel;
  typedef control_core::JointRef<const typename cc::traits<Derived>::Acc::Base> CAcc;
  typedef control_core::JointRef<const typename cc::traits<Derived>::Effort::Base> CEffort;

protected:
  Derived& ref_;
  int start_row_;
  int block_rows_;

public:

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
   * \param block_rows
   *      the start index of the column for Eigen::Block.
   */
  JointStateRef(Derived& ref, 
            int start_row,
            int block_rows) : 
    ref_(ref),
    start_row_(start_row),
    block_rows_(block_rows)
  {
  }

  Pos q()
  {
    return Pos(ref_.q(), start_row_, block_rows_);
  }

  Pos pos()
  {
    return Pos(ref_.q(), start_row_, block_rows_);
  }

  CPos q() const
  {
    return CPos(ref_.q(), start_row_, block_rows_);
  }

  CPos pos() const
  {
    return CPos(ref_.q(), start_row_, block_rows_);
  }

  Vel qP()
  {
    return Vel(ref_.qP(), start_row_, block_rows_);
  }

  Vel vel()
  {
    return Vel(ref_.qP(), start_row_, block_rows_);
  }

  CVel qP() const
  {
    return CVel(ref_.qP(), start_row_, block_rows_);
  }

  CVel vel() const
  {
    return CVel(ref_.qP(), start_row_, block_rows_);
  }

  Acc qPP()
  {
    return Acc(ref_.qPP(), start_row_, block_rows_);
  }

  Acc acc()
  {
    return Acc(ref_.qPP(), start_row_, block_rows_);
  }

  CAcc qPP() const
  {
    return CAcc(ref_.qPP(), start_row_, block_rows_);
  }

  CAcc acc() const
  {
    return CAcc(ref_.qPP(), start_row_, block_rows_);
  }

  Effort tau()
  {
    return Effort(ref_.effort(), start_row_, block_rows_);
  }

  Effort effort()
  {
    return Effort(ref_.effort(), start_row_, block_rows_);
  }

  CEffort tau() const
  {
    return CEffort(ref_.effort(), start_row_, block_rows_);
  }

  CEffort effort() const
  {
    return CEffort(ref_.effort(), start_row_, block_rows_);
  }

private:
  /*!
   * \brief No null reference.
   */
  JointStateRef();
};

}  // namespace control_core

#endif  // CONTROL_CORE_VECTOR_REF_H
