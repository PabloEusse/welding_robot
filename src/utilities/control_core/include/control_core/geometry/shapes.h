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

#ifndef CONTROL_CORE_GEOMETRY_SHAPES_H
#define CONTROL_CORE_GEOMETRY_SHAPES_H

#include <control_core/geometry/i_shape.h>
#include <control_core/geometry/geometry_2d.h>

// the namespace for the project
namespace cc
{
  /**
   * @brief Point Shape
   * 
   */
  class PointShape : public ShapeBase
  {
  public:
    PointShape() : ShapeBase(),
                   point_(cc::Vector2::Zero())
    {
    }

    PointShape(cc::Scalar x, cc::Scalar y) : 
      ShapeBase(),
      point_((cc::Vector2() << x, y).finished())
    {
    }

    PointShape(const cc::Vector2 &point) : 
      ShapeBase(), 
      point_(point)
    {
    }

    virtual ~PointShape()
    {
    }

    /**
     * @brief check the collision between shape and point
     * 
     * @param point 
     * @param min_dist 
     * @return true 
     * @return false 
     */
    bool checkCollision(const cc::Vector2 &point, cc::Scalar min_dist) const
    {
      return distance_points(point_, point) < min_dist;
    }

    /**
     * @brief return the minimum distance between shape and point
     * 
     * @param point 
     * @return cc::Scalar 
     */
    cc::Scalar minimumDistance(const cc::Vector2 &point) const
    {
      return distance_points(point_, point);
    }

    /**
     * @brief return the minimum distance between shape and polygon
     * 
     * @param polygon 
     * @return cc::Scalar 
     */
    cc::Scalar minimumDistance(const std::vector<cc::Vector2> &polygon) const
    {
      return distance_point_polygon(point_, polygon);
    }

    /**
     * @brief return the closest point on the shape
     * 
     * @param point 
     * @return const cc::Vector2 
     */
    const cc::Vector2 closestPoint(const cc::Vector2 &point) const
    {
      return point_;
    }

    /**
     * @brief get the position
     * 
     * @return const cc::Vector2& 
     */
    const cc::Vector2 &position() const
    {
      return point_;
    }

    /**
     * @brief convert to a ros visualization maker msg
     * 
     * @return visualization_msgs::Marker 
     */
    virtual visualization_msgs::Marker toMarkerMsg(
      cc::Scalar r=0.0, 
      cc::Scalar g = 0.0, 
      cc::Scalar b = 0.0, 
      cc::Scalar a = 1.0,
      cc::Scalar scale = 0.05)
    {
      visualization_msgs::Marker marker;
      marker.type = visualization_msgs::Marker::POINTS;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = a;
      marker.scale.x = scale;
      marker.scale.y = scale;

      geometry_msgs::Point point;
      point.x = point_.x();
      point.y = point_.y();
      marker.points.push_back(point);

      return marker;
    }

  private:
    cc::Vector2 point_;
  };

  /**
   * @brief line like obstacle
   * 
   */
  class LineShape : public ShapeBase
  {
  public:
    LineShape() : ShapeBase(),
                  s_(cc::Vector2::Zero()),
                  e_(cc::Vector2::Zero())
    {
    }

    /**
     * @brief Construct a new Line Shape object
     * 
     * @param s 
     * @param e 
     */
    LineShape(const cc::Vector2 &s, const cc::Vector2 &e) : ShapeBase(),
                                                            s_(s),
                                                            e_(e)
    {
    }

    /**
     * @brief Construct a new Line Shape object
     * 
     * @param line 
     */
    LineShape(const LineShape &line) : ShapeBase(),
                                       s_(line.s_),
                                       e_(line.e_)
    {
    }

    virtual ~LineShape()
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
    bool checkCollision(const cc::Vector2 &point, cc::Scalar min_dist) const
    {
      return distance_point_line_segment(point, s_, e_) < min_dist;
    }

    /**
     * @brief return the minimum distance between shape and point
     * 
     * @param point 
     * @return cc::Scalar 
     */
    cc::Scalar minimumDistance(const cc::Vector2 &point) const
    {
      return distance_point_line_segment(point, s_, e_);
    }

    /**
     * @brief return the minimum distance between shape and polygon
     * 
     * @param polygon 
     * @return cc::Scalar 
     */
    cc::Scalar minimumDistance(const std::vector<cc::Vector2> &polygon) const
    {
      return distance_line_polygon(s_, e_, polygon);
    }

    /**
     * @brief return the closest point on the shape
     * 
     * @param point 
     * @return const cc::Vector2 
     */
    const cc::Vector2 closestPoint(const cc::Vector2 &position) const
    {
      return closest_point_on_line_segment(position, s_, e_);
    }

    /**
     * @brief fit the shape to given set of points
     * 
     * @param points 
     */
    void fit(const std::vector<cc::Vector2>& points)
    {
      fit_line_segment(points, s_, e_);
    }

    /**
     * @brief get the start position
     * 
     * @return const cc::Vector2& 
     */
    const cc::Vector2 &start() const
    {
      return s_;
    }

    /**
     * @brief get the end position
     * 
     * @return const cc::Vector2& 
     */
    const cc::Vector2 &end() const
    {
      return e_;
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
      cc::Scalar scale = 0.05)
    {
      visualization_msgs::Marker marker;
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = a;
      marker.scale.x = scale;

      geometry_msgs::Point point;
      point.x = s_.x();
      point.y = s_.y();
      marker.points.push_back(point);
      point.x = e_.x();
      point.y = e_.y();
      marker.points.push_back(point);

      return marker;
    }

  private:
    cc::Vector2 s_;
    cc::Vector2 e_;
  };

  /**
   * @brief Circle Shape
   * 
   */
  class CircleShape : public ShapeBase
  {
  public:
    /**
     * @brief Construct a new Circle Shape object
     * 
     */
    CircleShape() : ShapeBase(),
                    center_(cc::Vector2::Zero()),
                    radius_(0.0)
    {
    }

    /**
     * @brief Construct a new Circle Shape object
     * 
     * @param center 
     * @param radius 
     */
    CircleShape(const cc::Vector2 &center, cc::Scalar radius) : ShapeBase(),
                                                                center_(center),
                                                                radius_(radius)
    {
    }

    /**
     * @brief Construct a new Circle Shape object
     * 
     * @param circle 
     */
    CircleShape(const CircleShape &circle) : ShapeBase(),
                                             center_(circle.center_),
                                             radius_(circle.radius_)
    {
    }

    /**
     * @brief Destroy the Circle Shape object
     * 
     */
    virtual ~CircleShape()
    {
    }

    /**
     * @brief check the collision between shape and point
     * 
     * @param point 
     * @param min_dist 
     * @return true 
     * @return false 
     */
    bool checkCollision(const cc::Vector2 &point, cc::Scalar min_dist) const
    {
      return ((point - center_).norm() - radius_) < min_dist;
    }

    /**
     * @brief return the minimum distance between shape and point
     * 
     * @param point 
     * @return cc::Scalar 
     */
    cc::Scalar minimumDistance(const cc::Vector2 &point) const
    {
      return distance_points(center_, point) - radius_;
    }

    /**
     * @brief return the minimum distance between shape and polygon
     * 
     * @param polygon 
     * @return cc::Scalar 
     */
    cc::Scalar minimumDistance(const std::vector<cc::Vector2> &polygon) const
    {
      return distance_point_polygon(center_, polygon) - radius_;
    }

    /**
     * @brief return the closest point on the shape
     * 
     * @param point 
     * @return const cc::Vector2 
     */
    const cc::Vector2 closestPoint(const cc::Vector2 &point) const
    {
      return center_ + radius_ * (point - center_).normalized();
    }

    /**
     * @brief fit the shape to given set of points
     * 
     * @param points 
     */
    void fit(const std::vector<cc::Vector2>& points)
    {
      fit_circle(points, center_, radius_);
    }

    /**
     * @brief get the radius
     * 
     * @return cc::Scalar 
     */
    cc::Scalar r() const
    {
      return radius_;
    }

    /**
     * @brief return the center
     * 
     * @return const cc::Vector2& 
     */
    const cc::Vector2 &c() const
    {
      return center_;
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
      cc::Scalar scale = 0.05)
    {
      visualization_msgs::Marker marker;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = a;
      marker.scale.x = scale;

      geometry_msgs::Point point;
      for(size_t i = 0; i < 30; ++i)
      {
        point.x = radius_*cos(2*M_PI*cc::Scalar(i)/30.) + center_.x();
        point.y = radius_*sin(2*M_PI*cc::Scalar(i)/30.) + center_.y();
        marker.points.push_back(point);
      }
      marker.points.push_back(marker.points.front());
      return marker;
    }

  private:
    cc::Vector2 center_;
    cc::Scalar radius_;
  };

  /**
   * @brief Polygon Shape
   * 
   */
  class PolygonShape : public ShapeBase
  {
  public:
    /**
     * @brief Construct a new Polygon Shape object
     * 
     * @param vertices 
     */
    PolygonShape(const std::vector<cc::Vector2> &vertices) : ShapeBase(),
                                                             vertices_(vertices)
    {
    }

    /**
     * @brief Construct a new Polygon Shape object
     * 
     * @param polygon 
     */
    PolygonShape(const PolygonShape &polygon) : ShapeBase(),
                                                vertices_(polygon.vertices_)
    {
    }

    /**
     * @brief Destroy the Polygon Shape object
     * 
     */
    virtual ~PolygonShape()
    {
    }

    /**
     * @brief check the collision between shape and point
     * 
     * @param point 
     * @param min_dist 
     * @return true 
     * @return false 
     */
    bool checkCollision(const cc::Vector2 &point, cc::Scalar min_dist) const
    {
      int i, j;
      bool c = false;

      // check if inside
      for (i = 0, j = vertices_.size() - 1; i < vertices_.size(); j = i++)
      {
        if (((vertices_[i].y() > point.y()) != (vertices_[j].y() > point.y())) &&
            (point.x() < (vertices_[j].x() - vertices_[i].x()) * (point.y() - vertices_[i].y()) / (vertices_[j].y() - vertices_[i].y()) + vertices_[i].x()))
        {
          c = !c;
        }
      }

      if (c > 0)
      {
        // inside
        return true;
      }

      if (min_dist == 0)
      {
        // on the edge
        return false;
      }

      // outside
      return minimumDistance(point) < min_dist;
    };

    /**
     * @brief return the minimum distance between shape and point
     * 
     * @param point 
     * @return cc::Scalar 
     */
    cc::Scalar minimumDistance(const cc::Vector2 &point) const
    {
      return distance_point_polygon(point, vertices_);
    };

    /**
     * @brief return the minimum distance between shape and polygon
     * 
     * @param polygon 
     * @return cc::Scalar 
     */
    cc::Scalar minimumDistance(const std::vector<cc::Vector2> &polygon) const
    {
      return distance_polygon_polygon(polygon, vertices_);
    }

    /**
     * @brief return the closest point on the shape
     * 
     * @param point 
     * @return const cc::Vector2 
     */
    const cc::Vector2 closestPoint(const cc::Vector2 &point) const
    {
      // a point
      if (vertices_.size() == 1)
      {
        return vertices_.front();
      }

      // a line
      if (vertices_.size() > 1)
      {
        cc::Vector2 pt = closest_point_on_line_segment(point, vertices_.front(), vertices_.back());

        // a polygon
        if (vertices_.size() > 2)
        {
          cc::Scalar dmin = (pt - point).norm();
          cc::Vector2 pt_min = pt;

          for (size_t i = 1; i < vertices_.size() - 1; ++i)
          {
            pt = closest_point_on_line_segment(point, vertices_[i], vertices_[i - 1]);
            cc::Scalar d = (pt - point).norm();
            if (d < dmin)
            {
              dmin = d;
              pt_min = pt;
            }
          }
          return pt_min;
        }
        return pt;
      }

      // error case
      return cc::Vector2::Zero();
    };

    /**
     * @brief Get the vertices
     * 
     * @return const std::vector<cc::Vector2>& 
     */
    const std::vector<cc::Vector2> &vertices() const
    {
      return vertices_;
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
      cc::Scalar scale = 0.05)
    {
      visualization_msgs::Marker marker;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = a;
      marker.scale.x = scale;

      geometry_msgs::Point point;
      for(size_t i = 0; i < vertices_.size(); ++i) 
      {
          point.x = vertices_[i].x();
          point.y = vertices_[i].y();
          marker.points.push_back(point);
      }
      if (vertices_.size() > 2) 
      {
          point.x = vertices_.front().x();
          point.y = vertices_.front().y();
          marker.points.push_back(point);
      }
      return marker;
    }

  private:
    std::vector<cc::Vector2> vertices_;
  };

} // namespace cc

#endif