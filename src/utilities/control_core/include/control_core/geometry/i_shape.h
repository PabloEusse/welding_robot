/*! \file
 *
 * \author Emmanuel Dean-Leon
 * \author Florian Bergner
 * \author J. Rogelio Guadarrama-Olvera
 * \author Simon Armleder
 * \author Gordon Cheng
 *
 * \version 0.1
 * \date 22.03.2021
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
 * #### Acknowledgment
 *  This project has received funding from the European Union‘s Horizon 2020
 *  research and innovation programme under grant agreement No 732287.
 */

#ifndef CONTROL_CORE_GEOMETRY_ISHAPES_H
#define CONTROL_CORE_GEOMETRY_ISHAPES_H

#include <control_core/types.h>
#include <visualization_msgs/Marker.h>

// the namespace for the project
namespace cc
{
  /**
   * @brief The Base class for all shapes
   * 
   */
  class ShapeBase
  {
  public:
    /**
     * @brief Construct a new Shape Base object
     * 
     */
    ShapeBase()
    {
    }

    /**
     * @brief Destroy the Shape Base object
     * 
     */
    virtual ~ShapeBase()
    {
    }

    /**
     * @brief check the collision between shape and point
     * 
     * @param position 
     * @param min_dist 
     * @return true 
     * @return false 
     */
    virtual bool checkCollision(const cc::Vector2 &point, cc::Scalar min_dist) const = 0;

    /**
     * @brief return the minimum distance between shape and point
     * 
     * @param position 
     * @return cc::Scalar 
     */
    virtual cc::Scalar minimumDistance(const cc::Vector2 &position) const = 0;

    /**
     * @brief return the minimum distance between shape and polygon
     * 
     * @param polygon 
     * @return cc::Scalar 
     */
    virtual cc::Scalar minimumDistance(const std::vector<Vector2> &polygon) const = 0;

    /**
     * @brief return the closest point on the shape
     * 
     * @param point 
     * @return const cc::Vector2 
     */
    virtual const cc::Vector2 closestPoint(const cc::Vector2 &point) const = 0;

    /**
     * @brief fit the shape to given set of points
     * 
     * @param points 
     */
    virtual void fit(const std::vector<cc::Vector2>& points)
    {
    }

    /**
     * @brief convert to a ros visualization maker msg
     * 
     * @return visualization_msgs::Marker 
     */
    virtual visualization_msgs::Marker toMarkerMsg(
      cc::Scalar r = 0.0, 
      cc::Scalar g = 0.0, 
      cc::Scalar b = 0.0, 
      cc::Scalar a = 1.0,
      cc::Scalar scale = 0.05) = 0; 
  };

}

#endif