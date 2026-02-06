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

#ifndef CONTROL_CORE_STATE_TRAJECTORIES_H
#define CONTROL_CORE_STATE_TRAJECTORIES_H

#include <control_core/types.h>
#include <geometry_msgs/PoseStamped.h>

#include <control_core/trajectory/polynomial_trajectories.h>
#include <control_core/trajectory/state_trajectory.h>

#include <control_core/math/quaternion.h>

/**
 * @brief This file contains some useful definition for common state trajectoreis
 * 
 */

namespace cc
{

/**
 * @brief A fith order (minimal jerk) trajectory in jointspace
 */
class JointStateSpline : public control_core::StateTrajectory<cc::JointState>
{
public:
  typedef control_core::StateTrajectory<cc::JointState> Base;

public:
  JointStateSpline(cc::Scalar period, const cc::JointPosition& q_start, const cc::JointPosition& q_end, int size=Eigen::Dynamic) :
    Base(control_core::Polynomial5Order<cc::JointPosition>(period, 
      q_start, cc::JointPosition::Zero(q_start.size()), cc::JointPosition::Zero(q_start.size()),
      q_end, cc::JointPosition::Zero(q_start.size()), cc::JointPosition::Zero(q_start.size()))) {}
};

/**
 * @brief A fith order (minimal jerk) trajectory for positions
 */
class LinearStateSpline : public control_core::StateTrajectory<cc::LinearState>
{
public:
  typedef control_core::StateTrajectory<cc::LinearState> Base;

public:
  LinearStateSpline(cc::Scalar period, const cc::LinearPosition& x_start, const cc::LinearPosition& x_end) :
    Base(control_core::Polynomial5Order<cc::LinearPosition>(period, 
      x_start, cc::LinearPosition::Zero(), cc::LinearPosition::Zero(),
      x_end, cc::LinearPosition::Zero(), cc::LinearPosition::Zero())) {}
};

/**
 * @brief A Angular Motion between two Quaternions using slerp interpolation
 */
class AngularStateSlerpTrajectory
{
public:
  typedef cc::Scalar Scalar;
  typedef std::vector<Scalar> ScalarVec;
  typedef control_core::AngularState<Scalar> State;
  typedef std::vector<AngularState> StateVec;

  typedef control_core::AngularPosition<Scalar> Position;
  typedef control_core::AngularVelocity<Scalar> Velocity;
  typedef control_core::AngularAcceleration<Scalar> Acceleration;
  typedef control_core::Polynomial<Scalar> Polynomial;

private:
  Scalar period_;
  Position Q_start_;
  Position Q_end_;
  Polynomial time_law_;

public:
  /**
   * @brief Construct a new Angular State Slerp object
   * 
   * @param period 
   * @param Q_start 
   * @param Q_end 
   */
  AngularStateSlerpTrajectory(Scalar period, const Position& Q_start, const Position& Q_end) : 
    period_(period),
    Q_start_(Q_start),
    Q_end_(Q_end),
    time_law_(Polynomial::Polynomial5Order(period, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0))
  {
  }

  /*!
   * @brief Deconstructor.
   */
  ~AngularStateSlerpTrajectory()
  {
  }

  /*!
   * @brief get end time of the trajectory
   */
  Scalar startTime() const
  {
    return 0.0;
  }

  /*!
   * @brief get end time of a segment
   */
  Scalar endTime() const
  {
    return period_;
  }

  /*!
   * @brief evaluate trajectory at a single time t
   *  
   * @return The state [pos, vel, acc] at time t
   */
  State evaluate(const Scalar& t) const
  {
    State state;

    Polynomial::ValueVec s;
    if(t <= period_)
    {
      s = time_law_.evaluateAll(t);
    }
    else
    {
      s = {0.0, 0.0, 1.0};
    }

    // compute Quaternion and derviatives w.r.t timelaw s
    state.pos() = cc::slerp(Q_start_, Q_end_, s[2]);
    Position dQds = cc::slerpP(Q_start_, Q_end_, s[2]);
    Position ddQds = cc::slerpPP(Q_start_, Q_end_, s[2]);

    // compute Quaternion time derviatives w.r.t time t
    Position QP, QPP;
    QP.coeffs() = dQds.coeffs()*s[1];
    QPP.coeffs() = dQds.coeffs()*s[0] + ddQds.coeffs()*s[1];

    // convert to angular velocites and acceleration
    state.vel() = cc::omegaWorld(state.pos(), QP);
    state.acc() = cc::alphaWorld(state.pos(), QPP);
    return state;
  }

  /*!
   * @brief evaluate trajectory at a all time t_vec
   *  
   * @return The state [pos, vel, acc] all time t_vec
   */
  StateVec evaluate(const ScalarVec& t_vec) const
  {
    StateVec states;

    states.resize(t_vec.size());
    for(size_t i = 0; i < states.size(); ++i)
    {
      states[i] = evaluate(t_vec[i]);
    }
    return states;
  }
};

/**
 * @brief A Cartesian Motion between two Cartesian poses using slerp interpolation
 */
class CartesianStateSlerpTrajectory
{
public:
  typedef cc::Scalar Scalar;
  typedef std::vector<Scalar> ScalarVec;
  typedef control_core::CartesianState<Scalar> State;
  typedef std::vector<CartesianState> StateVec;

  typedef control_core::CartesianPosition<Scalar> Position;
  typedef control_core::CartesianVelocity<Scalar> Velocity;
  typedef control_core::CartesianAcceleration<Scalar> Acceleration;
  typedef control_core::Polynomial<Scalar> Polynomial;

private:
  Scalar period_;
  Position X_start_;
  Position X_end_;
  Polynomial time_law_;

public:
  /**
   * @brief Construct a new Angular State Slerp object
   * 
   * @param period 
   * @param Q_start 
   * @param Q_end 
   */
  CartesianStateSlerpTrajectory(Scalar period, const Position& X_start, const Position& X_end) : 
    period_(period),
    X_start_(X_start),
    X_end_(X_end),
    time_law_(Polynomial::Polynomial5Order(period, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0))
  {
  }

  /*!
   * @brief Deconstructor.
   */
  ~CartesianStateSlerpTrajectory()
  {
  }

  /*!
   * @brief get end time of the trajectory
   */
  Scalar startTime() const
  {
    return 0.0;
  }

  /*!
   * @brief get end time of a segment
   */
  Scalar endTime() const
  {
    return period_;
  }

  /*!
   * @brief evaluate trajectory at a single time t
   *  
   * @return The state [pos, vel, acc] at time t
   */
  State evaluate(const Scalar& t) const
  {
    State state;

    Polynomial::ValueVec s;
    if(t <= period_)
    {
      s = time_law_.evaluateAll(t);
    }
    else if(t < 0.0)
    {
      s = {0.0, 0.0, 0.0};
    }
    else if(t > period_)
    {
      s = {0.0, 0.0, 1.0};
    }

    // compute position and its derviatives w.r.t timelaw s
    state.pos().linear() = X_start_.linear() + s[2]*(X_end_.linear() - X_start_.linear());
    cc::LinearPosition dxds = X_end_.linear() - X_start_.linear();
    cc::LinearPosition ddxds = cc::LinearPosition::Zero();

    // compute position derviatives w.r.t time t
    state.vel().linear() = dxds*s[1];
    state.acc().linear() = dxds*s[0] + ddxds*s[1];

    // compute Quaternion and its derviatives w.r.t timelaw s
    state.pos().angular() = cc::slerp(X_start_.angular(), X_end_.angular(), s[2]);
    cc::AngularPosition dQds = cc::slerpP(X_start_.angular(), X_end_.angular(), s[2]);
    cc::AngularPosition ddQds = cc::slerpPP(X_start_.angular(), X_end_.angular(), s[2]);

    // compute Quaternion derviatives w.r.t time t
    cc::AngularPosition QP, QPP;
    QP.coeffs() = dQds.coeffs()*s[1];
    QPP.coeffs() = dQds.coeffs()*s[0] + ddQds.coeffs()*s[1];

    // convert to angular velocites and acceleration
    state.vel().angular() = cc::omegaWorld(state.pos().angular(), QP);
    state.acc().angular() = cc::alphaWorld(state.pos().angular(), QPP);
    return state;
  }

  /*!
   * @brief evaluate trajectory at a all time t_vec
   *  
   * @return The state [pos, vel, acc] all time t_vec
   */
  StateVec evaluate(const ScalarVec& t_vec) const
  {
    StateVec states;

    states.resize(t_vec.size());
    for(size_t i = 0; i < states.size(); ++i)
    {
      states[i] = evaluate(t_vec[i]);
    }
    return states;
  }

  /**
   * @brief Convert to nav_msgs::Path with n_steps evenly spaced in time.
   * 
   * @param n_steps 
   * @return nav_msgs::Path 
   */
  std::vector<geometry_msgs::PoseStamped> toNavPath(size_t n_steps)
  {
    std::vector<geometry_msgs::PoseStamped> path;
    geometry_msgs::PoseStamped pose;

    double dt_eval = (endTime() - startTime())/cc::Scalar(n_steps);

    for(size_t i = 0; i < n_steps; ++i)
    {
      cc::Scalar time = startTime() + i*dt_eval;
      pose.pose = evaluate(time).pos();
      path.push_back(pose);
    }
    return path;
  }
};



}

#endif