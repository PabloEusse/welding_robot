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


#ifndef CONTROL_CORE_LINEAR_STATE_H
#define CONTROL_CORE_LINEAR_STATE_H


#include <control_core/types/linear_position.h>
#include <control_core/types/linear_velocity.h>
#include <control_core/types/linear_acceleration.h>
#include <control_core/types/force.h>

#include <control_core/type_bases/linear_state_base.h>

namespace cc{

/*!
 * \brief The traits class for the LinearState class.
 *
 * This class contains the typedefs and enums for
 * the LinearState class.
 *
 * These typedefs and enums are used to define and describe
 * the properties (traits) of this class.
 */
template<typename _Scalar>
struct traits<control_core::LinearState<_Scalar> >
{
  typedef _Scalar Scalar;
  typedef control_core::LinearPosition<Scalar> Pos;
  typedef control_core::LinearVelocity<Scalar> Vel;
  typedef control_core::LinearAcceleration<Scalar> Acc;
  typedef control_core::Force<Scalar> Effort;
  enum
  {
    IsRef = 0,
  };
};

} // namespace cc

namespace control_core{

/*!
 * \brief The LinearState class.
 *
 *  This class is a container for:
 *    - LinearPosition
 *    - LinearVelocity
 *    - LinearAcceleration
 *    - Force
 */
template<typename _Scalar>
class LinearState :
  public LinearStateBase<LinearState<_Scalar> >
{
public:
  typedef _Scalar Scalar;
  typedef LinearStateBase<LinearState<Scalar> > Base;

  typedef LinearPosition<Scalar> Pos;
  typedef LinearVelocity<Scalar> Vel;
  typedef LinearAcceleration<Scalar> Acc;
  typedef Force<Scalar> Effort;

protected:
  Pos x_;
  Vel xP_;
  Acc xPP_;
  Effort f_;

public:
  /*!
    * \brief Default Constructor.
    */
  LinearState()
  {
  }

  /*!
   * \brief Copy constructor.
   */
  LinearState(const LinearState& other) :
    x_(other.x_),
    xP_(other.xP_),
    xPP_(other.xPP_),
    f_(other.f_)
  {
  }

  /*!
   * \brief Copy constructor.
   */
  LinearState(const Base& other) : 
    x_(other.pos()),
    xP_(other.vel()),
    xPP_(other.acc()),
    f_(other.effort())
  {
  }

  /*!
    * \brief Constructor from sub elements
    */
  explicit LinearState(
    const Pos& x, 
    const Vel& xP = Vel::Zero(), 
    const Acc& xPP = Acc::Zero(), 
    const Effort& f = Effort::Zero()) :
    x_(x),
    xP_(xP),
    xPP_(xPP),
    f_(f)
  {
  }

  Pos& x()
  {
    return x_;
  }

  Pos& pos()
  {
    return x_;
  }

  const Pos& x() const
  {
    return x_;
  }

  const Pos& pos() const
  {
    return x_;
  }

  Vel& xP()
  {
    return xP_;
  }

  Vel& vel()
  {
    return xP_;
  }

  const Vel& xP() const
  {
    return xP_;
  }

  const Vel& vel() const
  {
    return xP_;
  }
  
  Acc& xPP()
  {
    return xPP_;
  }

  Acc& acc()
  {
    return xPP_;
  }

  const Acc& xPP() const
  {
    return xPP_;
  }

  const Acc& acc() const
  {
    return xPP_;
  }

  Effort& f()
  {
    return f_;
  }

  Effort& effort()
  {
    return f_;
  }

  const Effort& f() const
  {
    return f_;
  }

  const Effort& effort() const
  {
    return f_;
  }

  /*!
   * \brief Use assignment operators of base class.
   */
  using Base::operator=;

  /*!
   * \brief Assignment operator.
   */
  LinearState& operator=(const LinearState& other)
  {
    Base::operator=(other);
    return *this;
  }

};


}

#endif // CONTROL_CORE_LINEAR_STATE_H
