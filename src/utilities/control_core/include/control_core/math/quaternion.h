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

#ifndef CONTROL_CORE_MATH_QUATERNION_H
#define CONTROL_CORE_MATH_QUATERNION_H

#include <control_core/types.h>

/*! 
 * \file quaternion.h
 * \brief Contains functions for quaternion related math
 */

// the namespace for the project
namespace cc{

/*!
 * \brief Flip Quaternions sign if scalar part w < 0.
 *
 * Flip Quaternion if scalar part sign smaller than zero, to prevent
 * pi/2 ambiguity in computation of the quaternion log map.
 */
template <typename Derived1>
inline bool checkFlipQuaternionSign(control_core::QuaternionBase<Derived1>& Q)
{
  if(Q.derived().w() < 0.0)
  {
    Q.derived().coeffs() *= -1.0;
    return true;
  }
  return false;
}

/**
 * @brief computes the quaternion matrix function S(3): H -> R^4x4
 * 
 * Eq. (109)
 * 
 * @tparam Derived 
 * @param omega 
 * @return Matrix4 
 */
template <typename Derived>
inline Matrix4 QMatrix(const control_core::QuaternionBase<Derived>& Q)
{
  const Derived& Q_ = Q.derived();
  return (Matrix4() << 
    Q_.w(), -Q_.x(), -Q_.y(), -Q_.z(), 
    Q_.x(),  Q_.w(),  Q_.z(), -Q_.y(), 
    Q_.y(), -Q_.z(),  Q_.w(),  Q_.x(), 
    Q_.z(),  Q_.y(), -Q_.x(),  Q_.w()).finished();
}

/**
 * @brief computes the conjugate quaternion matrix function S(3): H -> R^4x4
 * 
 * Eq. (109)
 * 
 * @tparam Derived 
 * @param omega 
 * @return Matrix4 
 */
template <typename Derived>
inline Matrix4 QConjugateMatrix(const control_core::QuaternionBase<Derived>& Q)
{
  const Derived& Q_ = Q.derived();
  return (Matrix4() << 
    Q_.w(), -Q_.x(), -Q_.y(), -Q_.z(), 
    Q_.x(),  Q_.w(), -Q_.z(),  Q_.y(), 
    Q_.y(),  Q_.z(),  Q_.w(), -Q_.x(), 
    Q_.z(), -Q_.y(),  Q_.x(),  Q_.w()).finished();
}

/**
 * @brief map a quaternion and its temporal derivative to the angular velocity 
 * in world coordinates
 * 
 * @tparam Derived1 
 * @tparam Derived2 
 * @param Q Quaternion of body wrt world
 * @param QP Quaternion coeff time derivative
 * @return AngularVelocity of body expressed in world frame
 */
template <typename Derived1, typename Derived2>
inline AngularVelocity omegaWorld(    
  const control_core::QuaternionBase<Derived1>& Q,
  const control_core::QuaternionBase<Derived2>& QP)
{
  return Scalar(2)*QMatrix(Q).transpose().bottomRows(3)*QP.derived().coeffs();
}

/**
 * @brief map a quaternion and its temporal derivative to the angular velocity 
 * in body coordinates
 * 
 * @tparam Derived1 
 * @tparam Derived2 
 * @param Q Quaternion of body wrt world
 * @param QP Quaternion coeff time derivative
 * @return AngularVelocity of body expressed in body frame 
 */
template <typename Derived1, typename Derived2>
inline AngularVelocity omegaBody(    
  const control_core::QuaternionBase<Derived1>& Q,
  const control_core::QuaternionBase<Derived2>& QP)
{
  return Scalar(2)*QConjugateMatrix(Q).transpose().bottomRows(3)*QP.derived().coeffs();
}

/**
 * @brief map a quaternion and its second derivative to the angular acceleration in world coordinates
 * 
 * @tparam Derived1 
 * @tparam Derived2 
 * @param Q Quaternion of body wrt world
 * @param QP Quaternion coeff time derivative
 * @return AngularAcceleration of body expressed in body world  
 */
template <typename Derived1, typename Derived2>
inline AngularAcceleration alphaWorld(    
  const control_core::QuaternionBase<Derived1>& Q,
  const control_core::QuaternionBase<Derived2>& QPP)
{
  return omegaWorld(Q, QPP);
}

/**
 * @brief map a quaternion and its second derivative to the angular acceleration in body coordinates
 * 
 * @tparam Derived1 
 * @tparam Derived2 
 * @param Q Quaternion of body wrt world
 * @param QP Quaternion coeff time derivative
 * @return AngularAcceleration of body expressed in body   
 */
template <typename Derived1, typename Derived2>
inline AngularAcceleration alphaBody(    
  const control_core::QuaternionBase<Derived1>& Q,
  const control_core::QuaternionBase<Derived2>& QPP)
{
  return omegaBody(Q, QPP);
}

/**
 * @brief integrate body angular velocity to quaternion
 * 
 * @tparam Derived1 
 * @tparam Derived2 
 * @param Q_b_w Quaternion of body wrt world
 * @param omega_b angular velocity of body expressed in body 
 * @param dt time step
 * @return AngularPosition of body wrt world
 */
template <typename Derived1, typename Derived2>
inline AngularPosition integrateOmegaBody(
  const control_core::QuaternionBase<Derived1>& Q_b_w,
  const control_core::AngularVelocityBase<Derived2>& omega_b, cc::Scalar dt)
{
  return AngularPosition::ExpMap(omega_b.derived()*dt)*Q_b_w.derived();
}

/**
 * @brief integrate body angular velocity to quaternion                         
 * 
 * @tparam Derived1 
 * @tparam Derived2 
 * @param Q_b_w Quaternion of body wrt world
 * @param omega_w angular velocity of body expressed in world 
 * @param dt time step
 * @return AngularPosition of body wrt world 
 */
template <typename Derived1, typename Derived2>
inline AngularPosition integrateOmegaWorld(
  const control_core::QuaternionBase<Derived1>& Q_b_w,
  const control_core::AngularVelocityBase<Derived2>& omega_w, cc::Scalar dt)
{
  return AngularPosition::ExpMap(omega_w.derived()*dt)*Q_b_w.derived();         // TODO: should be fliped? I think so!
}

/**
 * @brief comptue the motion vector between two quaternions in the world frame
 * 
 * @tparam Derived1 
 * @tparam Derived2 
 * @param Qd desired quaternion body wrt world
 * @param Q current quaternion body wrt world
 * @return Vector3 motion vector expressed in world 
 */
template <typename Derived1, typename Derived2>
inline Vector3 logErrorWorld(                                                   // TODO: and for 
  const control_core::QuaternionBase<Derived1>& Qd,
  const control_core::QuaternionBase<Derived2>& Q)
{
  typedef cc::Scalar Scalar;

  // Relative quaternion between desired Q and current Q
  control_core::AngularPosition<Scalar> Qe = Qd.derived()*Q.derived().inverse();
  checkFlipQuaternionSign(Qe);

  // return the log map of the delta quaternion
  return Qe.logMap();
}

/**
 * @brief comptue the motion vector between two quaternions in the body frame
 * 
 * @tparam Derived1 
 * @tparam Derived2 
 * @param Qd desired quaternion body wrt world
 * @param Q current quaternion body wrt world
 * @return Vector3 motion vector expressed in body  
 */
template <typename Derived1, typename Derived2>
inline Vector3 logErrorLocal(                                                  
  const control_core::QuaternionBase<Derived1>& Qd,
  const control_core::QuaternionBase<Derived2>& Q)
{
  typedef cc::Scalar Scalar;

  // Relative quaternion between desired Q and current Q
  control_core::AngularPosition<Scalar> Qe = Q.derived().inverse()*Qd.derived();
  checkFlipQuaternionSign(Qe);

  // return the log map of the delta quaternion
  return Qe.logMap();
}

/**
 * @brief Spherical linear interpolation between two Quaternions
 * 
 * @tparam Derived1 
 * @tparam Derived2 
 * @param Qi Inital Quaternion
 * @param Qf Final Quaternion
 * @param s interplation parameter
 * @return AngularPosition Interpolated Quaternion
 */
template <typename Derived1, typename Derived2>
inline AngularPosition slerp(
  const control_core::QuaternionBase<Derived1>& Qi,
  const control_core::QuaternionBase<Derived2>& Qf,
  cc::Scalar s)
{
  typedef cc::Scalar Scalar;
  const Scalar one = Scalar(1) - std::numeric_limits<Scalar>::epsilon();

  cc::AngularPosition Qs;
  Scalar d = Qi.derived().dot(Qf.derived());
  Scalar d_abs = std::abs(d);

  Scalar scale_0;
  Scalar scale_1;

  if(d_abs >= one)
  {
    // linear interpolation
    scale_0 = Scalar(1) - s;
    scale_1 = s;
  }
  else
  {
    // spherical interpolation
    Scalar th = std::acos(d_abs);
    Scalar th_sin = std::sin(th);

    scale_0 = std::sin((Scalar(1) - s)*th)/th_sin;
    scale_1 = std::sin(s*th)/th_sin;
  }
  if(d < Scalar(0))
  {
    // take the shortest path
    scale_1 = -scale_1;
  }
  Qs.coeffs() = scale_0*Qi.derived().coeffs()
      + scale_1*Qf.derived().coeffs();
  return Qs;
}

/*!
 * \brief First derivative of SLERP
 *
 * First Order Derivate of interpolation between
 * intial and final AngularPosition wrt to parameterization s.
 * 
 * \note This is the derivative of the quaternion coefficients, not 
 * the time derivative (angular velocities).
 *
 * QsP = d slerp(Qi,Qf,s) / ds
 *
 * See: https://en.wikipedia.org/wiki/Slerp
 */
template <typename Derived1, typename Derived2>
inline AngularPosition slerpP(
    const control_core::QuaternionBase<Derived1>& Qi,
    const control_core::QuaternionBase<Derived2>& Qf,
    cc::Scalar s)
{
  typedef cc::Scalar Scalar;
  const Scalar one = Scalar(1) - std::numeric_limits<Scalar>::epsilon();

  AngularPosition QsP;
  Scalar d = Qi.derived().dot(Qf.derived());
  Scalar d_abs = std::abs(d);

  Scalar scale_0;
  Scalar scale_1;

  if(d_abs >= one)
  {
    // linear interpolation
    scale_0 = -1;
    scale_1 = 1;
  }
  else
  {
    // spherical interpolation
    Scalar th = std::acos(d_abs);
    Scalar th_sin = std::sin(th);

    scale_0 = -th*std::cos((Scalar(1) - s)*th)/th_sin;
    scale_1 = th*std::cos(s*th)/th_sin;
  }
  if(d < Scalar(0))
  {
    // take the shortest path
    scale_1 = -scale_1;
  }
  QsP.coeffs() = scale_0*Qi.derived().coeffs()
      + scale_1*Qf.derived().coeffs();
  return QsP;
}

/*!
 * \brief Second derivative of SLERP
 *
 * Second Order Derivate of interpolation between
 * intial and final AngularPosition wrt to parameterization s.
 *
 * QsPP = d2 slerp(Qi,Qf,s) / ds2
 *
 * See: https://en.wikipedia.org/wiki/Slerp
 */
template <typename Derived1, typename Derived2>
inline AngularPosition slerpPP(
    const control_core::QuaternionBase<Derived1>& Qi,
    const control_core::QuaternionBase<Derived2>& Qf,
    cc::Scalar s)
{
  typedef cc::Scalar Scalar;
  const Scalar one = Scalar(1) - std::numeric_limits<Scalar>::epsilon();

  AngularPosition QsPP;
  Scalar d = Qi.derived().dot(Qf.derived());
  Scalar d_abs = std::abs(d);

  Scalar scale_0;
  Scalar scale_1;

  if(d_abs >= one)
  {
    // linear interpolation
    scale_0 = 0;
    scale_1 = 0;
  }
  else
  {
    // spherical interpolation
    Scalar th = std::acos(d_abs);
    Scalar th_sin = std::sin(th);

    scale_0 = -th*th*std::sin((Scalar(1) - s)*th)/th_sin;
    scale_1 = -th*th*std::sin(s*th)/th_sin;
  }
  if(d < Scalar(0))
  {
    // take the shortest path
    scale_1 = -scale_1;
  }
  QsPP.coeffs() = scale_0*Qi.derived().coeffs()
      + scale_1*Qf.derived().coeffs();
  return QsPP;
}

} // namespace cc

#endif // CONTROL_CORE_MATH_QUATERNION_H