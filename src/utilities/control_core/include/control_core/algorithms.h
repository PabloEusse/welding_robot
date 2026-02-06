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


#ifndef CONTROL_CORE_ALGORITHMS_H
#define CONTROL_CORE_ALGORITHMS_H

/*! \file types.h
 *  \brief Contains all the algorithms.
 */

// get the configurations to correctly define all algorithms
#include <control_core/configuration.h>

#include <control_core/algorithms/scalar_finite_difference.h>
#include <control_core/algorithms/scalar_butterworth_filter.h>
#include <control_core/algorithms/scalar_exponential_decay_filter.h>

#include <control_core/algorithms/state_differentiator.h>
#include <control_core/algorithms/state_integrator.h>

// the namespace for the configured algorithms
namespace cc
{
  typedef control_core::ScalarFiniteDifference<cc::Scalar> 
    ScalarFiniteDifference;

  typedef control_core::ScalarButterWorthFilter<cc::Scalar> 
    ScalarButterWorthFilter;

  typedef control_core::ScalarExponentialDecayFilter<cc::Scalar> 
    ScalarExponentialDecayFilter;

  template<typename _T>
  using StateIntegrator = control_core::StateIntegrator<_T>;
  template<typename _T>
  using StateDifferentiator = control_core::StateDifferentiator<_T>;
}

#endif  // CONTROL_CORE_ALGORITHMS_H
