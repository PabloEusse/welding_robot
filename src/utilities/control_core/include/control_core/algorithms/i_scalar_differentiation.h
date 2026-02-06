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


#ifndef CONTROL_CORE_I_SCALAR_DIFFERENTIATION_H
#define CONTROL_CORE_I_SCALAR_DIFFERENTIATION_H

#include <control_core/algorithms/i_scalar_algorithm.h>

namespace control_core{

/*!
 * \brief The IScalarDifferentiation class.
 *
 * This class describes the interface for scalar differentiation 
 * algorithms
 */
template <typename _Scalar>
class IScalarDifferentiation :
  public IScalarAlgorithm<_Scalar>
{
public:
  typedef _Scalar Scalar;

public:
  /*!
   * \brief Virtual destructor.
   */
  virtual ~IScalarDifferentiation()
  {
  }

  /*!
   * \brief The step width of the computed derivative.
   *
   * \returns the step width \f$\mathScalar{h}\f$.
   *
   */
  virtual Scalar stepWidth() const = 0;

  /*!
   * \brief The order of the computed derivative.
   *
   * \returns the order \f$\mathScalar{d}\f$ of the computed derivative.
   * E.g. the first derivative is of order 1.
   *
   */
  virtual int order() const = 0;

  /*!
   * \brief The accuracy of the computed derivative.
   *
   * \returns the order \f$\mathScalar{a}\f$ of the computed derivative.
   *
   */
  virtual int accuracy() const = 0;

  /*!
   * \brief the signal sampling frequency.
   */
  virtual Scalar sampleFrequency() const = 0;

};

} // namespace control_core

#endif // CONTROL_CORE_I_DIFFERENTIATION_ALGORITHM_H
