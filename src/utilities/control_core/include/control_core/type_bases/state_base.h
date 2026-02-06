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


#ifndef CONTROL_CORE_STATE_BASE_H
#define CONTROL_CORE_STATE_BASE_H

#include <control_core/utilities/support_templates.h>
#include <control_core/utilities/forward_declarations.h>

namespace control_core
{

template <typename _Derived, int _IsRef>
struct StateBaseTraits;

template <typename _Derived>
struct StateBaseTraits<_Derived, 0>
{
  typedef typename cc::traits<_Derived>::Pos& Pos;
  typedef typename cc::traits<_Derived>::Vel& Vel;
  typedef typename cc::traits<_Derived>::Acc& Acc;
  typedef typename cc::traits<_Derived>::Effort& Effort;
  typedef const typename cc::traits<_Derived>::Pos& CPos;
  typedef const typename cc::traits<_Derived>::Vel& CVel;
  typedef const typename cc::traits<_Derived>::Acc& CAcc;
  typedef const typename cc::traits<_Derived>::Effort& CEffort;
};

template <typename _Derived>
struct StateBaseTraits<_Derived, 1>
{
  typedef typename cc::traits<_Derived>::Pos Pos;
  typedef typename cc::traits<_Derived>::Vel Vel;
  typedef typename cc::traits<_Derived>::Acc Acc;
  typedef typename cc::traits<_Derived>::Effort Effort;
  typedef const typename cc::traits<_Derived>::CPos CPos;
  typedef const typename cc::traits<_Derived>::CVel CVel;
  typedef const typename cc::traits<_Derived>::CAcc CAcc;
  typedef const typename cc::traits<_Derived>::CEffort CEffort;
};

/*!
 * \brief The StateBase class.
 *
 * Template interface class for the group of state type classes:
 * JointState, LinearState, AngularState, CartesianState.
 *
 * This class is important when implementing operations that
 * should be available for all the state classes.
 */
template <typename _Derived>
class StateBase
{
public:
  typedef _Derived Derived;

  typedef typename cc::traits<Derived>::Pos Pos;
  typedef typename cc::traits<Derived>::Vel Vel;
  typedef typename cc::traits<Derived>::Acc Acc;
  typedef typename cc::traits<Derived>::Effort Effort;
  enum
  {
    IsRef = cc::traits<Derived>::IsRef,
  };

  typedef typename StateBaseTraits<Derived, IsRef>::Pos RetPos;
  typedef typename StateBaseTraits<Derived, IsRef>::Vel RetVel;
  typedef typename StateBaseTraits<Derived, IsRef>::Acc RetAcc;
  typedef typename StateBaseTraits<Derived, IsRef>::Effort RetEffort;
  typedef typename StateBaseTraits<Derived, IsRef>::CPos CRetPos;
  typedef typename StateBaseTraits<Derived, IsRef>::CVel CRetVel;
  typedef typename StateBaseTraits<Derived, IsRef>::CAcc CRetAcc;
  typedef typename StateBaseTraits<Derived, IsRef>::CEffort CRetEffort;

public:

  /*!
   * \brief Get Zero State for static Sizes
   */
  static const Derived& Zero()
  {
    static Derived v;
    static bool once = false;
    if(!once)
    {
      v.pos() = Pos::Zero();
      v.vel() = Vel::Zero();
      v.acc() = Acc::Zero();
      v.effort() = Effort::Zero();
      once = true;
    }
    return v;
  }

  /*!
   * \brief Get Zero State for dynamic Sizes
   */
  template<typename T>
  static const Derived& Zero(const T& x)
  {
    static Derived v;
    v.pos() = Pos::Zero(x);
    v.vel() = Vel::Zero(x);
    v.acc() = Acc::Zero(x);
    v.effort() = Effort::Zero(x);
    return v;
  }

public:

  /*!
   * \brief Get accces to top level class.
   */
  Derived& derived()
  {
    return *static_cast<Derived*>(this);
  }

  /*!
   * \brief Get accces to top level class.
   */
  const Derived& derived() const
  {
    return *static_cast<const Derived*>(this);
  }

  /*!
   * \brief Assignment operator.
   */
  StateBase& operator=(const StateBase& other)
  {
    derived().pos() = other.pos();
    derived().vel() = other.vel();
    derived().acc() = other.acc();
    derived().effort() = other.effort();
    return *this;
  }

  /*!
   * \brief set everything to zero.
   */
  StateBase& setZero()
  {
    derived().pos().setZero();
    derived().vel().setZero();
    derived().acc().setZero();
    derived().effort().setZero();
    return *this;
  }

  /*!
   * \brief set everything to zero.
   */
  template<typename T>
  StateBase& setZero(const T& x)
  {
    derived().pos().setZero(x);
    derived().vel().setZero(x);
    derived().acc().setZero(x);
    derived().effort().setZero(x);
    return *this;
  }

  /*!
   * \brief resize the vector.
   */
  template<typename T>
  void resize(const T& x)
  {
    derived().pos().resize(x);
    derived().vel().resize(x);
    derived().acc().resize(x);
    derived().effort().resize(x);
  }

  /*!
   * \brief General Position value.
   */
  RetPos pos()
  {
    return derived().pos();
  }

  /*!
   * \brief General Position value.
   */
  CRetPos pos() const
  {
    return derived().pos();
  }

  /*!
   * \brief General Velocity value.
   */
  RetVel vel()
  {
    return derived().vel();
  }

  /*!
   * \brief General Velocity value.
   */
  CRetVel vel() const
  {
    return derived().vel();
  }

  /*!
   * \brief General Acceleration value.
   */
  RetAcc acc()
  {
    return derived().acc();
  }

  /*!
   * \brief General Acceleration value.
   */
  CRetAcc acc() const
  {
    return derived().acc();
  }

  /*!
   * \brief General Force value.
   */
  RetEffort effort()
  {
    return derived().effort();
  }

  /*!
   * \brief General Force value.
   */
  CRetEffort effort() const
  {
    return derived().effort();
  }

  /*!
   * \brief General Print Function.
   */
  std::string toString() const
  {
    std::ostringstream out;
    out << "pos    = [" << pos().toString() << "]\n";
    out << "vel    = [" << vel().toString() << "]\n";
    out << "acc    = [" << acc().toString() << "]\n";
    out << "effort = [" << effort().toString() << "]\n";

    return out.str();
  }
  
};

} // namespace control_core

#endif  // CONTROL_CORE_STATE_BASE_H
