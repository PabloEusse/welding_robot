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


#ifndef CONTROL_CORE_JACOBAIN_H
#define CONTROL_CORE_JACOBAIN_H

#include <control_core/type_references/jacobian_ref.h>
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
template <typename _Scalar, int _Cols>
struct traits<control_core::Jacobian<_Scalar, _Cols> >
{
  typedef _Scalar Scalar;
  enum
  {
    RowsAtCompileTime = 6,
    ColsAtCompileTime = _Cols
  };
};

} // namespace cc


namespace control_core
{
/*!
 * \brief The Jacobian class.
 *
 * The Jacobian is of type Eigen::Matrix and is
 * represented by the math symbol \f$\mathbf{W}\f$.
 *
 * Stores the force and moment in a 6 dimensional vector.
 * The force is represented by the first three elements.
 * The moment velocity by the last three elements.
 *
 */
template <typename _Scalar, int _Cols>
class Jacobian :
    public Eigen::Matrix<_Scalar, 6, _Cols>,
    public JacobianBase<Jacobian<_Scalar, _Cols> >,
    TypeGuard
{
  //OW_TYPE_GUARD(Jacobian)

public:
  typedef _Scalar Scalar;
  typedef Eigen::Matrix<Scalar, 6, _Cols> Base;
  typedef JacobianBase<Jacobian<_Scalar, _Cols> > JBase;

  typedef control_core::JacobianRef<Base, _Cols> LinearRef;
  typedef control_core::JacobianRef<Base, _Cols> AngularRef;
  typedef control_core::JacobianRef<const Base, _Cols> CLinearRef;
  typedef control_core::JacobianRef<const Base, _Cols> CAngularRef;

  /*!
   * \brief Construct as Default
   *
   * Default is Zero.
   */
  static const Jacobian& Default()
  {
    static const Jacobian v = Base::Zero();
    return v;
  }

public:
  /*!
   * \brief Default Constructor
   */
  Jacobian()
  {
  }

  /*!
   * \brief Copy Constructor
   */
  Jacobian(const Jacobian& other) : 
    Base(other)
  {
  }

  /*!
   * \brief Copy Constructor
   */
  Jacobian(const JBase& other) :
    JBase(other)
  {
  }

  /*!
   * \brief Copy constructor.
   *
   * This copy constructor not only works with Eigen matrices
   * but also with their expressions.
   */
  template <typename OtherDerived>
  Jacobian(const Eigen::EigenBase<OtherDerived>& other) :
    Base(other)
  {
  }

  /*!
   * \brief Use assignment operators of base class.
   */
  using Base::operator=;
  using JBase::operator=;

  /*!
   * \brief Assignment operator.
   */
  const Jacobian& operator=(const Jacobian& other)
  {
    Base::operator=(other);
    return *this;
  }

  /*!
   * \brief access to linear part
   */
  LinearRef linear()
  {
    return LinearRef(Base::derived(), 0);
  }

  /*!
   * \brief const access to linear part
   */
  CLinearRef linear() const
  {
    return CLinearRef(Base::derived(), 0);
  }

  /*!
   * \brief access to angular part
   */
  AngularRef angular()
  {
    return AngularRef(Base::derived(), 3);
  }

  /*!
   * \brief const access to angular part
   */
  CAngularRef angular() const
  {
    return CAngularRef(Base::derived(), 3);
  }

};

}  // namespace control_core

#endif  // CONTROL_CORE_WRENCH_REF_H
