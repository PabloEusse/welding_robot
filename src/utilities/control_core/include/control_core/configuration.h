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


#ifndef CONTROL_CORE_CONFIGURATION_H
#define CONTROL_CORE_CONFIGURATION_H


/*! \file configuration.h
 *  \brief Contains the global configurations.
 */

/*! \def CC_ROBOT_DOF
 *  \brief A definition to set degrees of freedom of the robot.
 */
#define CC_ROBOT_DOF                      6
#define CC_ARM_DOF                        6
#define CC_MOBILE_BASE_DOF                0 // not used here!

/*! \def CC_TYPES_SCALAR
 *  \brief A definition to set the scalar type for all type classes.
 */
#define CC_TYPES_SCALAR             double

/*! \def CC_VECTOR_DOF_SCALAR
 *  \brief A definition to set the scalar type of the VectorDof.
 *
 *  Either set it to CC_TYPES_SCALAR or a builtin type.
 *
 *  \note For now the system only supports a common type
 *  for CC_TYPES_SCALAR and CC_VECTOR_DOF_SCALAR.
 */
#define CC_VECTOR_DOF_SCALAR        CC_TYPES_SCALAR

/*! \def CC_GRAVITY
 *  \brief Magnitude of gravity [m/s^2].
 */
#define CC_GRAVITY                  9.81


#endif  // CONTROL_CORE_CONFIGURATION_H
