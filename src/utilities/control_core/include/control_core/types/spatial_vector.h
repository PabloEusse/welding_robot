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


#ifndef CONTROL_CORE_SPATIAL_VECTOR_H
#define CONTROL_CORE_SPATIAL_VECTOR_H

#include <control_core/utilities/type_guard.h>
#include <control_core/type_bases/cartesian_base.h>

namespace cc{

/*!
 * \brief The traits class for the SpatialVector class.
 *
 * This class contains the typedefs and enums for
 * the SpatialVector class.
 *
 * These typedefs and enums are used to define and describe
 * the properties (traits) of this class.
 */
template<typename _Scalar>
struct traits<control_core::SpatialVector<_Scalar> >
{
  typedef Eigen::Matrix<_Scalar, 6, 1> Base;
  typedef control_core::Vector3Ref<Base> LinearRef;
  typedef control_core::Vector3Ref<Base> AngularRef;
  typedef control_core::Vector3Ref<const Base> CLinearRef;
  typedef control_core::Vector3Ref<const Base> CAngularRef;
  enum {
    LinearPartAtCompileTime = 3,
    AngularPartCompileTime = 0,
  };
};

} // namespace cc

namespace control_core
{
/*!
 * \brief The SpatialVector class.
 *
 * The SpatialVector is of type CartesianBase.
 * 
 * Can be used to assign Spatial Types our ow types.
 */
template <typename _Scalar>
class SpatialVector :
  public Eigen::Matrix<_Scalar, 6, 1>,
  public CartesianBase<SpatialVector<_Scalar> >,
  TypeGuard
{
  //OW_TYPE_GUARD(SpatialVector)

public:
  typedef _Scalar Scalar;
  typedef Eigen::Matrix<_Scalar,6,1> Base;
  typedef CartesianBase<SpatialVector<Scalar> > CBase;

public:
  /*!
   * \brief Default Constructor.
   */
  SpatialVector() :
    Base()
  {
  }

  /*!
   * \brief Copy constructor.
   */
  SpatialVector(const SpatialVector& other) :
    Base(other)
  {
  }

  /*!
   * \brief Copy constructor.
   * 
   */
  SpatialVector(const CartesianVelocity<Scalar>& other)
  {
    CBase::operator=(other);
  }

  /*!
   * \brief Copy constructor.
   * 
   */
  SpatialVector(const CartesianAcceleration<Scalar>& other)
  {
    CBase::operator=(other);
  }

  /*!
   * \brief Copy constructor.
   * 
   */
  SpatialVector(const Wrench<Scalar>& other)
  {
    CBase::operator=(other);
  }

  /*!
   * \brief Copy constructor.
   * 
   */
  SpatialVector(const CartesianVector<Scalar>& other)
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
  SpatialVector(const Eigen::EigenBase<OtherDerived>& other) :
    Base(other)
  {
  }

  /*!
   * \brief Use assignment operators of VBase class.
   */
  using Base::operator=;
  using CBase::operator=;

  /*!
   * \brief Assignment from SpatialVector.
   */
  SpatialVector& operator=(const SpatialVector& other)
  {
    Base::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment from SpatialVector.
   *
   */
  SpatialVector& operator=(const CartesianVelocity<Scalar>& other)
  {
    CBase::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment from SpatialVector.
   *
   */
  SpatialVector& operator=(const CartesianAcceleration<Scalar>& other)
  {
    CBase::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment from SpatialVector.
   *
   */
  SpatialVector& operator=(const Wrench<Scalar>& other)
  {
    CBase::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment from CartesianVector.
   *
   */
  SpatialVector& operator=(const CartesianVector<Scalar>& other)
  {
    CBase::operator=(other);
    return *this;
  }
};

}  // namespace control_core


#endif  // CONTROL_CORE_VECTOR_DOF_H
