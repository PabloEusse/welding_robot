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


#ifndef CONTROL_CORE_LINEAR_STATE_BASE_H
#define CONTROL_CORE_LINEAR_STATE_BASE_H

#include <control_core/type_bases/state_base.h>
#include <control_core_msgs/LinearState.h>

namespace control_core{

/*!
 * \brief The LinearStateBase class.
 * 
 * \note this Class simply passes _Derived through to the StateBase
 */
template <typename _Derived>
class LinearStateBase :
  public StateBase<_Derived>
{
public:
  typedef _Derived Derived;
  typedef StateBase<Derived> Base;

public:
  using Base::operator=;

  /*!
   * \brief Assignment of control_core_msgs::LinearState.
   */
  LinearStateBase& operator=(const control_core_msgs::LinearState& msg)
  {
    Base::derived().pos() = msg.position;
    Base::derived().vel() = msg.velocity;
    Base::derived().acc() = msg.acceleration;
    Base::derived().effort() = msg.force;
    return *this;
  }

  /*!
   * \brief Conversion to control_core_msgs::LinearState.
   */
  operator control_core_msgs::LinearState() const
  {
    control_core_msgs::LinearState msg;
    msg.position = Base::derived().pos();
    msg.velocity = Base::derived().vel();
    msg.acceleration = Base::derived().acc();
    msg.force = Base::derived().effort();
    return msg;
  }  

  /*!
   * \brief Conversion to control_core_msgs::LinearState.
   */
  control_core_msgs::LinearState toLinearStateMsg() const
  {
    return static_cast<control_core_msgs::LinearState>(Base::derived());
  }
};

}  // namespace control_core

#endif  // CONTROL_CORE_LINEAR_STATE_BASE_H
