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


#ifndef CONTROL_CORE_CARTESIAN_VECTOR_H
#define CONTROL_CORE_CARTESIAN_VECTOR_H

#include <control_core/utilities/type_guard.h>
#include <control_core/type_bases/cartesian_base.h>

namespace cc{

/*!
 * \brief The traits class for the CartesianVector class.
 *
 * This class contains the typedefs and enums for
 * the CartesianVector class.
 *
 * These typedefs and enums are used to define and describe
 * the properties (traits) of this class.
 */
template<typename _Scalar>
struct traits<control_core::CartesianVector<_Scalar> >
{
  typedef Eigen::Matrix<_Scalar, 6, 1> Base;
  typedef control_core::Vector3Ref<Base> LinearRef;
  typedef control_core::Vector3Ref<Base> AngularRef;
  typedef control_core::Vector3Ref<const Base> CLinearRef;
  typedef control_core::Vector3Ref<const Base> CAngularRef;
  enum {
    LinearPartAtCompileTime = 0,
    AngularPartCompileTime = 3,
  };
};

} // namespace cc

namespace control_core
{
/*!
 * \brief The CartesianVector class.
 *
 * The CartesianVector is of type CartesianBase.
 * 
 * Can be used to store vectors in cartesian space. 
 * Having a linear and Angular parts.
 */
template <typename _Scalar>
class CartesianVector :
  public Eigen::Matrix<_Scalar, 6, 1>,
  public CartesianBase<CartesianVector<_Scalar> >,
  TypeGuard
{
  //OW_TYPE_GUARD(CartesianVector)

public:
  typedef _Scalar Scalar;
  typedef Eigen::Matrix<_Scalar,6,1> Base;
  typedef CartesianBase<CartesianVector<Scalar> > CBase;

public:
  /*!
   * \brief Default Constructor.
   */
  CartesianVector() :
    Base()
  {
  }

  /*!
   * \brief Copy constructor.
   */
  CartesianVector(const CartesianVector& other) :
    Base(other)
  {
  }

  /*!
   * \brief Copy constructor.
   * 
   */
  CartesianVector(const CartesianVelocity<Scalar>& other)
  {
    CBase::operator=(other);
  }

  /*!
   * \brief Copy constructor.
   * 
   */
  CartesianVector(const CartesianAcceleration<Scalar>& other)
  {
    CBase::operator=(other);
  }

  /*!
   * \brief Copy constructor.
   * 
   */
  CartesianVector(const Wrench<Scalar>& other)
  {
    CBase::operator=(other);
  }

  /*!
   * \brief Copy constructor.
   * 
   */
  CartesianVector(const SpatialVector<Scalar>& other)
  {
    CBase::operator=(other);
  }

  /*!
   *  \brief Copy constructor.
   *
   * This copy constructor not only works with Eigen matrices
   * but also with their expressions.
   */
  template<typename OtherDerived>
  CartesianVector(const Eigen::EigenBase<OtherDerived>& other) :
    Base(other)
  {
  }

  /*!
   * \brief Use assignment operators of VBase class.
   */
  using Base::operator=;
  using CBase::operator=;

  /*!
   * \brief Assignment from CartesianVector.
   */
  CartesianVector& operator=(const CartesianVector& other)
  {
    Base::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment from CartesianVector.
   *
   */
  CartesianVector& operator=(const CartesianVelocity<Scalar>& other)
  {
    CBase::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment from CartesianVector.
   *
   */
  CartesianVector& operator=(const CartesianAcceleration<Scalar>& other)
  {
    CBase::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment from CartesianVector.
   *
   */
  CartesianVector& operator=(const Wrench<Scalar>& other)
  {
    CBase::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment from SpatialVector.
   *
   */
  CartesianVector& operator=(const SpatialVector<Scalar>& other)
  {
    CBase::operator=(other);
    return *this;
  }
};

}  // namespace control_core


#endif  // CONTROL_CORE_CARTESIAN_VECTOR_H
