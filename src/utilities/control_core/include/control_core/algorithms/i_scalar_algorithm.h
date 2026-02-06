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


#ifndef CONTROL_CORE_I_SCALAR_ALGORITHM_H
#define CONTROL_CORE_I_SCALAR_ALGORITHM_H

#include <Eigen/Dense>

namespace control_core{


/*!
 * \brief The IScalarAlgorithm class.
 *
 * The interface class for scalar algorithms.
 * This class describes the interface for all single input single
 * output (SISO) algorithms.
 * 
 */
template <typename _Scalar>
class IScalarAlgorithm
{
public:
  typedef _Scalar Scalar;

public:
  /*!
   * \brief Virtual destructor.
   */
  virtual ~IScalarAlgorithm()
  {
  }

  /*!
   * \brief Returns whether the ouput is valid
   * 
   * Return true if algorithm accumulated enouth samples to fill buffer.
   */
  virtual bool valid() const = 0;

  /*!
   * \brief Returns num of samples required for valid ouput
   * 
   * Return number of samples required for valid ouput.
   */
  virtual int validCount() const = 0;

  /*!
   * \brief Create a copy of this object using the new operator.
   *
   * Use this function to get a new object of the implemented
   * differentiation algorithm. This function returns a pointer
   * and the caller of this function has to manage the allocated
   * memory on the heap. When the object is no longer needed
   * the caller has to delete it.
   */
  virtual IScalarAlgorithm* copy() const = 0;

  /*!
   * \brief Reset the algorithm to inital state.
   *
   * Ouput is initalized set in next call to update()
   */
  virtual Scalar reset() = 0;

  /*!
   * \brief Reset the algorithm to inital state \f$\mathScalarQ{x}{}\f$.
   *
   * Algorithm uses \f$\mathScalarQ{x}{}\f$ as inital value.
   */
  virtual Scalar reset(Scalar x) = 0;

  /*!
   * \brief Adds new sample \f$\mathScalarQ{x}{}\f$ wihtout computation.
   * 
   * Could be used to populate buffer, before algorithm is used by update.
   */
  virtual void add(Scalar x) = 0;

  /*!
   * \brief Update the algorithm by adding new sample \f$\mathScalarQ{x}{}\f$.
   *
   * Takes the new sample \f$\mathScalarQ{x}{}\f$ and
   * computes and new ouput \f$\mathScalarQ{y}{}\f$.
   */
  virtual Scalar update(Scalar x) = 0;

  /*!
   * \brief Returns the last ouput \f$\mathScalarQ{y}{}\f$.
   */
  virtual Scalar output() const = 0;

};

} // namespace control_core

#endif // CONTROL_CORE_I_SCALAR_ALGORITHM_H
