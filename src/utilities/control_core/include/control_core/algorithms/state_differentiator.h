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


#ifndef CONTROL_CORE_STATE_DIFFERENTIATOR_H
#define CONTROL_CORE_STATE_DIFFERENTIATOR_H

#include <ros/assert.h>

#include <control_core/types.h>
#include <control_core/utilities/type_not_assignable.h>

#include <control_core/math/quaternion.h>
#include <control_core/algorithms/matrix_algorithm.h>

namespace control_core{


/*!
 * \brief The StateDifferentiator class.
 *
 * This class takes the current position of a state class and
 * updates the current velocity and accelerations.
 *
 * The _Derived template parameter has to be derived from
 * control_core::StateBase thus belong to the group of
 * state type classes.
 */
template <typename _Derived>
class StateDifferentiator
{
  OW_TYPE_NOT_ASSIGNABLE(StateDifferentiator)

public:
  typedef _Derived Derived;

  typedef typename cc::traits<Derived>::Scalar Scalar;
  typedef typename cc::traits<Derived>::Pos V;
  typedef typename cc::traits<Derived>::Vel VP;
  typedef typename cc::traits<Derived>::Acc VPP;
  typedef typename cc::traits<Derived>::Effort VF;

  typedef IScalarDifferentiation<Scalar> IDiff;

protected:
  /*!
   * \brief The differentation for velocities.
   */
  MatrixAlgorithm<V> diff1_;

  /*!
   * \brief The differentation for accelerations.
   */
  MatrixAlgorithm<VP> diff2_;

public:
  /*!
   * \brief Constructor.
   *
   */
  StateDifferentiator(const IDiff& algo) :
    diff1_(algo),
    diff2_(algo)
  {
  }

  /*!
   * \brief Deconstructor.
   */
  ~StateDifferentiator()
  {
  }

  /*!
   * \brief Returns whether the ouput is valid
   * 
   * Return true if algorithm accumulated enougth samples to fill buffer.
   */
  bool valid() const 
  {
    return (diff1_.valid() && diff2_.valid()); 
  }

  /*!
   * \brief Reset the differentiation.
   *
   * Reset all differentiation algorithms.
   */
  Derived reset()
  {
    Derived ret = Derived::Zero();

    convertToVelocity(
      ret.vel(),
      diff1_.reset(),
      V::Zero());

    convertToAcceleration(
      ret.acc(),
      diff2_.reset(),
      VP::Zero());
  
    return ret;
  }

  /*!
   * \brief Reset the differentiation.
   *
   * Reset all differentiation algorithms to a given state
   */
  Derived reset(const Derived& state)
  {
    Derived ret;
    ret.pos() = state.pos();
    ret.effort() = state.effort();

    convertToVelocity(
      ret.vel(),
      diff1_.reset(state.pos()),
      V::Zero());

    convertToAcceleration(
      ret.acc(),
      diff2_.reset(state.vel()),
      VP::Zero());

    return ret;
  }

  /*!
   * \brief Adds new sample to algorithm
   *
   * Could be used to populate buffer, before algorithm is used by update.
   */
  void add(const Derived& state)
  {
    diff1_.add(state.pos());
    diff2_.add(state.vel());
  }

  /*!
   * \brief Update the differentiation for the next step.
   *
   * - Takes the new state \a state,
   * - Updates the history, and
   * - computes and updates the velocites and acceleration of the state.
   *
   * \returns the updated state.
   */
  Derived& update(Derived& state)
  {
    // compute the first derivative
    convertToVelocity(
      state.vel(),
      diff1_.update(state.pos()),
      state.pos());

    // compute the second derivative
    convertToAcceleration(
      state.acc(),
      diff2_.update(state.vel()),
      state.vel());

    return state;
  }

protected:
  /*!
  * \brief Conversion of Position Type to Velocity Type
  * 
  * This function covers the general case for which the velocity is 
  * directly given by the time derivative of the position.
  */
  template <typename Derived1, typename Derived2, typename Derived3>
  inline Eigen::MatrixBase<Derived1>& convertToVelocity(
    Eigen::MatrixBase<Derived1>& vel,
    const Eigen::MatrixBase<Derived2>& posP,
    const Eigen::MatrixBase<Derived3>& pos)
  {
    vel = posP;
    return vel;
  };

  /*!
  * \brief Conversion of Position Type to Velocity Type
  * 
  * This function covers the special case for AngularPositions to 
  * AngularVelocities. This requires the time derivative of the
  * AngularPosition and the current AngularPosition.
  */
  template <typename Derived1, typename Derived2>
  inline cc::AngularVelocity& convertToVelocity(
    cc::AngularVelocity& omega,
    const control_core::AngularPositionBase<Derived1>& QP,
    const control_core::AngularPositionBase<Derived2>& Q)
  {
    cc::quaternion2AngularVelocityInertial(omega, QP, Q);
    return omega;
  };

  /*!
  * \brief Conversion of Position Type to Velocity Type
  * 
  * This function covers the special case for CartesianPositions to 
  * CartesianVelocities. This requires the time derivative of the
  * CartesianPosition and the current CartesianPosition.
  */
  inline cc::CartesianVelocity& convertToVelocity(
    cc::CartesianVelocity& vel,
    const cc::CartesianPosition& XP,
    const cc::CartesianPosition& X)
  {
    vel.linear() = static_cast<const control_core::Vector3<Scalar>& >(
      XP.position());
    cc::quaternion2AngularVelocityInertial(
      vel.angular(), 
      XP.orientation(), 
      X.orientation());
    return vel;
  };

  /*!
  * \brief Conversion of Velocity Type to Acceleration Type
  * 
  * This function covers the general case for which the acceleration is 
  * directly given by the time derivative of the velocity.
  * \note there is no special case, everything is cartesian.
  */
  template <typename Derived1, typename Derived2, typename Derived3>
  inline Eigen::MatrixBase<Derived1>& convertToAcceleration(
    Eigen::MatrixBase<Derived1>& acc,
    const Eigen::MatrixBase<Derived2>& velP,
    const Eigen::MatrixBase<Derived3>& vel)
  {
    acc = velP;
    return acc;
  };
};

}

#endif // CONTROL_CORE_STATE_DERIVATIVES_UPDATER_H
