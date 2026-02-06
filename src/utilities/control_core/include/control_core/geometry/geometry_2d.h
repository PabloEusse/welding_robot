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

#ifndef CONTROL_CORE_GEOMETRY_GEOMETRY_2D
#define CONTROL_CORE_GEOMETRY_GEOMETRY_2D

#include <control_core/types.h>

// the namespace for the project
namespace cc
{

  /**
   * @brief cross product beetween two 2d vectors
   * 
   * @param p1 
   * @param p2 
   * @return cc::Scalar 
   */
  inline cc::Scalar angle_sign(const cc::Vector2 &p1, const cc::Vector2 &p2)
  {
    return p1.x() * p2.y() - p2.x() * p1.y();
  }

  /**
 * @brief distance between two points
 * 
 * @param p1 
 * @param p2 
 * @return cc::Scalar 
 */
  inline cc::Scalar distance_points(const cc::Vector2 &p1, const cc::Vector2 &p2)
  {
    return (p1 - p2).norm();
  }

  /**
   * @brief distance point p to parametric line: w^T + w0 = 0
   * 
   * parameter vector has the form [b, w1, w2]
   * 
   * @param p 
   * @param w
   * @return cc::Scalar 
   */
  inline cc::Scalar distance_point_line(const cc::Vector2 &p, const cc::Vector3 &w)
  {
    return std::abs(w.tail(2).dot(p) + w[0]) / w.tail(2).norm();
  }

  /**
   * @brief compute the closest point to p on parametric line: w^T + w0 = 0 
   * 
   * @param p 
   * @param w 
   * @return cc::Vector2 
   */
  inline cc::Vector2 closest_point_on_line(const cc::Vector2 &p, const cc::Vector3 &w)
  {
    cc::Scalar d = w[1] * w[1] + w[2] * w[2];

    if (d > 0.0)
    {
      cc::Vector2 cp;
      cp[0] = (w[2] * w[2] * p[0] - w[1] * w[2] * p[1] - w[1] * w[0]) / d;
      cp[1] = (-w[1] * w[2] * p[0] + w[1] * w[1] * p[1] - w[2] * w[0]) / d;
      return cp;
    }

    // error
    return p;
  }

  /**
   * @brief distance point p to line defined by start point s and end point e
   * 
   * @param p 
   * @param s 
   * @param e 
   * @return cc::Scalar 
   */
  inline cc::Scalar distance_point_line(
      const cc::Vector2 &p,
      const cc::Vector2 &s,
      const cc::Vector2 &e)
  {
    cc::Vector2 d = e - s;
    return (d.y() * p.x() - d.x() * p.y() + e.x() * s.y() - e.y() * s.x()) / d.norm();
  }

  /**
   * @brief projection of point p to the line defines by start point s and end point e
   * 
   * @param p 
   * @param s 
   * @param e 
   * @return cc::Vector2 
   */
  inline cc::Vector2 closest_point_on_line_segment(
      const cc::Vector2 &p, const cc::Vector2 &s, const cc::Vector2 &e,
      cc::Scalar *distance = NULL)
  {
    cc::Vector2 cp;
    cc::Vector2 d = e - s;
    cc::Scalar sq_n = d.squaredNorm();

    if (sq_n == 0)
    {
      if (distance)
      {
        *distance = 0.0;
      }
      return s; // no line segment
    }

    cc::Scalar u = ((p.x() - s.x()) * d.x() + (p.y() - s.y()) * d.y()) / sq_n;
    if (u <= 0)
    {
      cp = s;
    }
    else if (u >= 1)
    {
      cp = e;
    }
    else
    {
      cp = s + u * d;
    }
    if (distance)
    {
      *distance = (p - cp).norm();
    }
    return cp;
  }

  /**
   * @brief distance between point p and the line defines by start point s and end point e
   * 
   * @param p 
   * @param s 
   * @param e 
   * @return cc::Scalar 
   */
  inline cc::Scalar distance_point_line_segment(
      const cc::Vector2 &p, const cc::Vector2 &s, const cc::Vector2 &e)
  {
    return (p - closest_point_on_line_segment(p, s, e)).norm();
  }

  /**
 * @brief checks if two line segments (s1, e1) and (s2, e2) intersect
 * 
 * See http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
 * 
 * @param s1 
 * @param e1 
 * @param s2 
 * @param e2 
 * @param intersection 
 * @return true 
 * @return false 
 */
  inline bool check_line_segments_intersection(
      const cc::Vector2 &s1, const cc::Vector2 &e1,
      const cc::Vector2 &s2, const cc::Vector2 &e2,
      cc::Vector2 *intersection = NULL)
  {
    cc::Scalar s_numer, t_numer, denom, t;
    cc::Vector2 line1 = e1 - s1;
    cc::Vector2 line2 = e2 - s2;

    denom = line1.x() * line2.y() - line2.x() * line1.y();
    if (denom == 0)
    {
      return false; // Collinear
    }

    bool is_denom_pos = denom > 0;
    cc::Vector2 aux = s1 - s2;

    s_numer = line1.x() * aux.y() - line1.y() * aux.x();
    if ((s_numer < 0) == is_denom_pos)
    {
      return false; // No collision
    }

    t_numer = line2.x() * aux.y() - line2.y() * aux.x();
    if ((t_numer < 0) == is_denom_pos)
    {
      return false; // No collision
    }

    if (((s_numer > denom) == is_denom_pos) || ((t_numer > denom) == is_denom_pos))
    {
      return false; // No collision
    }

    // Otherwise collision detected
    t = t_numer / denom;
    if (intersection)
    {
      *intersection = s1 + t * line1;
    }
    return true;
  }

  /**
   * @brief distance between two line segments
   * 
   * @param s1 
   * @param e1 
   * @param s2 
   * @param e2 
   * @return cc::Scalar 
   */
  inline cc::Scalar distance_segment_segment(
      const cc::Vector2 &s1, const cc::Vector2 &e1,
      const cc::Vector2 &s2, const cc::Vector2 &e2)
  {
    if (check_line_segments_intersection(s1, e1, s2, e2))
    {
      return 0;
    }

    // check all 4 combinations
    cc::Scalar distances[4];
    distances[0] = distance_point_line_segment(s1, s2, e2);
    distances[1] = distance_point_line_segment(e1, s2, e2);
    distances[2] = distance_point_line_segment(s2, s1, e1);
    distances[3] = distance_point_line_segment(e2, s1, e1);

    // return minimum
    cc::Scalar d_min = std::numeric_limits<cc::Scalar>::max();
    for (size_t i = 0; i < 4; ++i)
    {
      d_min = distances[i] < d_min ? distances[i] : d_min;
    }
    return d_min;
  }

  /**
   * @brief clostest point on line segment (s2,e2) mesasured between two given line segments (s1,e1), (s2,e2)
   * 
   * @param s1 
   * @param e1 
   * @param s2 
   * @param e2 
   * @param distance 
   * @return cc::Vector2 
   */
  inline cc::Vector2 closest_point_on_line_segment(
      const cc::Vector2 &s1, const cc::Vector2 &e1,
      const cc::Vector2 &s2, const cc::Vector2 &e2,
      cc::Scalar *distance = NULL)
  {
    cc::Vector2 closest_point;
    if (check_line_segments_intersection(s1, e1, s2, e2, &closest_point))
    {
      if (distance)
      {
        *distance = 0;
      }
      return closest_point;
    }

    cc::Scalar d1, d2;
    cc::Vector2 p;
    closest_point = closest_point_on_line_segment(s1, s2, e2);
    d1 = (s1 - closest_point).norm();
    p = closest_point_on_line_segment(e1, s2, e2);
    d2 = (e1 - p).norm();

    if (d1 < d2)
    {
      if (distance)
      {
        *distance = d1;
      }
      return closest_point;
    }
    else
    {
      if (distance)
      {
        *distance = d2;
      }
      return p;
    }
  }

  /**
   * @brief scalar product between line segments ( perpendicular = 0, collinear == 1 )
   * 
   * @param s1 
   * @param e1 
   * @param s2 
   * @param e2 
   * @return cc::Scalar 
   */
  inline cc::Scalar colinearity_segment_segment(
      const cc::Vector2 &s1, const cc::Vector2 &e1,
      const cc::Vector2 &s2, const cc::Vector2 &e2)
  {
    cc::Vector2 p1 = (e1 - s1).normalized();
    cc::Vector2 p2 = (e2 - s2).normalized();
    return p1.dot(p2);
  }

  /**
   * @brief distance of point p to polygon defined by vertice points
   * 
   * @param p 
   * @param vertices 
   * @return cc::Scalar 
   */
  inline cc::Scalar distance_point_polygon(
      const cc::Vector2 &p, const std::vector<cc::Vector2> &vertices)
  {
    if (vertices.size() == 1)
    {
      return (p - vertices[0]).norm();
    }

    // check each edge
    cc::Scalar dmin = std::numeric_limits<cc::Scalar>::max();
    for (size_t i = 0; i < vertices.size() - 1; ++i)
    {
      cc::Scalar d = distance_point_line_segment(p, vertices[i], vertices[i - 1]);
      dmin = d < dmin ? d : dmin;
    }
    // if not a line check edge back to front
    if (vertices.size() > 2)
    {
      cc::Scalar d = distance_point_line_segment(p, vertices.back(), vertices.front());
      dmin = d < dmin ? d : dmin;
    }
    return dmin;
  }

  /**
   * @brief distance line segment (s, e) to polygon defined by vertice points
   * 
   * @param s 
   * @param e 
   * @param vertices 
   * @return cc::Scalar 
   */
  inline cc::Scalar distance_line_polygon(
      const cc::Vector2 &s, const cc::Vector2 &e,
      const std::vector<cc::Vector2> &vertices)
  {
    if (vertices.size() == 1)
    {
      return distance_point_line_segment(vertices.front(), s, e);
    }

    cc::Scalar dmin = std::numeric_limits<cc::Scalar>::max();
    for (size_t i = 0; i < vertices.size() - 1; ++i)
    {
      cc::Scalar d = distance_segment_segment(s, e, vertices[i], vertices[i + 1]);
      dmin = d < dmin ? d : dmin;
    }
    if (vertices.size() > 2)
    {
      cc::Scalar d = distance_segment_segment(s, e, vertices.back(), vertices.front());
      dmin = d < dmin ? d : dmin;
    }
    return dmin;
  }

  /**
   * @brief distance between two polygons defined by points in vertices1 and vertices2
   * 
   * @param vertices1 
   * @param vertices2 
   * @return cc::Scalar 
   */
  inline cc::Scalar distance_polygon_polygon(
      const std::vector<cc::Vector2> &vertices1,
      const std::vector<cc::Vector2> &vertices2)
  {
    if (vertices1.size() == 1)
    {
      return distance_point_polygon(vertices1.front(), vertices2);
    }

    cc::Scalar dmin = std::numeric_limits<cc::Scalar>::max();
    for (size_t i = 0; i < vertices1.size() - 1; ++i)
    {
      cc::Scalar d = distance_line_polygon(vertices1[i], vertices1[i + 1], vertices2);
      dmin = d < dmin ? d : dmin;
    }
    if (vertices1.size() > 2)
    {
      cc::Scalar d = distance_line_polygon(vertices1.back(), vertices1.front(), vertices2);
      dmin = d < dmin ? d : dmin;
    }
    return dmin;
  }

  /**
   * @brief compute the closest point on polygon defined by vertices from point p
   * 
   * @param point 
   * @param vertices 
   * @param distance 
   * @return cc::Vector2 
   */
  inline cc::Vector2 closest_point_on_polygon(
      const cc::Vector2 &p, const std::vector<cc::Vector2> &vertices,
      cc::Scalar *distance = NULL)
  {
    if (vertices.size() == 1)
    {
      return vertices[0];
    }

    // check each edge
    cc::Scalar dmin = std::numeric_limits<cc::Scalar>::max();
    cc::Vector2 closest_pt;

    for (size_t i = 0; i < vertices.size() - 1; ++i)
    {
      cc::Vector2 pt = closest_point_on_line_segment(p, vertices[i], vertices[i - 1]);
      cc::Scalar d = (pt - p).norm();
      if (d < dmin)
      {
        closest_pt = pt;
        dmin = d;
      }
    }
    // if not a line check edge back to front
    if (vertices.size() > 2)
    {
      cc::Vector2 pt = closest_point_on_line_segment(p, vertices.back(), vertices.front());
      cc::Scalar d = (pt - p).norm();
      if (d < dmin)
      {
        closest_pt = pt;
        dmin = d;
      }
    }
    if (distance)
    {
      *distance = dmin;
    }
    return closest_pt;
  }

  /**
   * @brief check intersection between point p and the rectable defined by corners
   * rect_min and rect_max
   * 
   * @param rect_min 
   * @param rect_max 
   * @param p 
   * @return true 
   * @return false 
   */
  inline bool point_rect_intersect(
      const cc::Vector2 &rect_min, const cc::Vector2 &rect_max, const cc::Vector2 &p)
  {
    return (p.x() > rect_min.x() && p.x() < rect_max.x() &&
            p.y() > rect_min.y() && p.y() < rect_max.y());
  }

  /**
   * @brief check intersection between point p and circle (center, radius)
   * 
   * @param center 
   * @param radius 
   * @param p 
   * @return true 
   * @return false 
   */
  inline bool point_circle_intersect(
      const cc::Vector2 &center, cc::Scalar radius, const cc::Vector2 &p)
  {
    return (center - p).norm() < radius;
  }

  /**
   * @brief fit a line (w^T*x + w0 = 0) through given points
   * 
   * parameter vector has the form [b, w1, w2]
   * 
   * @param begin 
   * @param end 
   * @param w 
   */
  inline void fit_line(
      std::vector<cc::Vector2>::const_iterator begin,
      std::vector<cc::Vector2>::const_iterator end,
      cc::Vector3 &w)
  {
    cc::Scalar sx, sy, sx_sq, sy_sq, sxy, denom;
    sx = sy = sx_sq = sy_sq = sxy = 0.0;

    for (std::vector<cc::Vector2>::const_iterator it = begin; it != end; ++it)
    {
      sx += it->x();
      sy += it->y();
      sx_sq += it->x() * it->x();
      sy_sq += it->y() * it->y();
      sxy += it->x() * it->y();
    }
    denom = sx_sq * sy_sq - sxy * sxy;
    w[0] = -1.0;
    w[1] = (sy_sq * sx - sxy * sy) / denom;
    w[2] = (sx_sq * sy - sxy * sx) / denom;
  }

  /**
   * @brief fit a line (w^T*x + w0 = 0) through given points
   * 
   * parameter vector has the form [b, w1, w2]
   * 
   * @param points 
   * @param w 
   */
  inline void fit_line(const std::vector<cc::Vector2> &points, cc::Vector3 &w)
  {
    fit_line(points.cbegin(), points.cend(), w);
  }

  /**
   * @brief fit a line segment to set of points
   * 
   * Fits a line segment (s, e) through points, given parameters w
   * Assumes the points in std::vector are ordered start to end 
   * 
   * @param begin 
   * @param end 
   * @param s 
   * @param e 
   */
  inline void fit_line_segment(
      std::vector<cc::Vector2>::const_iterator begin,
      std::vector<cc::Vector2>::const_iterator end,
      cc::Vector2 &s, cc::Vector2 &e)
  {
    // project start and end point on parametric line
    cc::Vector3 w;
    fit_line(begin, end, w);
    s = closest_point_on_line(*begin, w);
    e = closest_point_on_line(*(--end), w);
  }

  /**
   * @brief fit a line segment to set of points
   * 
   * Fits a line segment (s, e) through points, given parameters w
   * Assumes the points in std::vector are ordered start to end 
   * 
   * @param points 
   * @param s 
   * @param e 
   */
  inline void fit_line_segment(
      const std::vector<cc::Vector2> &points,
      cc::Vector2 &s, cc::Vector2 &e)
  {
    return fit_line_segment(points.cbegin(), points.cend(), s, e);
  }

  /**
   * @brief fit a circle to set of points
   * 
   * @param begin 
   * @param end 
   * @param c 
   * @param r 
   */
  inline void fit_circle(
      std::vector<cc::Vector2>::const_iterator begin,
      std::vector<cc::Vector2>::const_iterator end,
      cc::Vector2 &c, cc::Scalar &r)
  {
    std::vector<cc::Vector2>::const_iterator it;
    int n = std::distance(begin, end);

    // compute mean
    cc::Vector2 mu = cc::Vector2::Zero();
    for (it = begin; it != end; ++it)
    {
      mu += *it;
    }
    mu /= cc::Scalar(n);

    // remove mean, cummulate sums
    cc::Scalar u, v, uc, vc;
    cc::Scalar Suu, Suv, Suuu, Suvv, Svv, Svvv, Svuu;
    Suu = Suv = Suuu = Suvv = Svv = Svvv = Svuu = 0.0;
    for (it = begin; it != end; ++it)
    {
      u = it->x() - mu[0];
      v = it->y() - mu[1];
      Suu += u * u;
      Suv += u * v;
      Suuu += u * u * u;
      Suvv += u * v * v;
      Svv += v * v;
      Svvv += v * v * v;
      Svuu += v * u * u;
    }

    // solve lgs
    cc::Scalar b1, b2;
    b1 = 0.5 * (Suuu + Suvv);
    b2 = 0.5 * (Svvv + Svuu);
    vc = (b1 - Suu / Suv * b2) / (Suv - Suu * Svv / Suv);
    uc = (b2 - Svv * vc) / Suv;

    // compute parameters
    r = std::sqrt(uc * uc + vc * vc + (Suu + Svv) / n);
    c[0] = uc + mu[0];
    c[1] = vc + mu[1];
  }

  inline void fit_circle(
      const std::vector<cc::Vector2> &points,
      cc::Vector2 &c, cc::Scalar &r)
  {
    fit_circle(points.cbegin(), points.cend(), c, r);
  }

} // namespace cc

#endif