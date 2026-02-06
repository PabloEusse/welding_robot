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


#ifndef CONTROL_CORE_ROS_H
#define CONTROL_CORE_ROS_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <control_core/types.h>

namespace cc
{

/*!
 * \brief Publish a msg if there is a subsriber
 */
template<typename _Msg>
inline bool publish_if_subscribed(ros::Publisher& pub, const _Msg& msg)
{
  if(pub.getNumSubscribers())
  {
    pub.publish(msg);
    return true;
  }
  return false;
}

/*!
 * \brief Read a transformation
 */
inline bool listenTransformation(
    tf::TransformListener& listener, 
    const std::string& parent, 
    const std::string& child, 
    CartesianPosition& X_parent_child, 
    Scalar dur = 10.0,
    bool verbose=false)
{   
    if(parent == child)
    {
        X_parent_child.setIdentity();
        return true;
    }

    tf::StampedTransform transform;
    try 
    {
        listener.waitForTransform(child, parent, ros::Time(0), ros::Duration(dur));
        listener.lookupTransform(parent, child, ros::Time(0), transform);
    } 
    catch (const tf::TransformException& ex) 
    {
        if(verbose)
            ROS_ERROR_STREAM("listenTransformation(): Can't find transformation" << child << " w.r.t. " << parent);
        return false;
    }
    X_parent_child = transform; 
    return true;   
}

/*!
 * \brief Check if ros has parameter.
 */
inline bool has(const std::string& name, bool verbose=false)
{
  if(!ros::param::has(name)) 
  {
    if(verbose)
    {
      ROS_ERROR("has: parameter '%s' is missing.", name.c_str());
    }
    return false;
  }
  return true;
}

/*!
 * \brief load bool value
 */
inline bool load(const std::string& name, bool& val, bool verbose=false)
{
  if(!has(name, verbose))
  {
    return false;
  }
  if(!ros::param::get(name, val))
  {
    if(verbose)
    {
      ROS_ERROR("load: could not load '%s'.", name.c_str());
    }
    return false;
  }
  return true;
}

/*!
 * \brief load int value
 */
inline bool load(const std::string& name, int& val, bool verbose=false)
{
  if(!has(name, verbose))
  {
    return false;
  }
  if(!ros::param::get(name, val))
  {
    if(verbose)
    {
      ROS_ERROR("load: could not load '%s'.", name.c_str());
    }
    return false;
  }
  return true;
}

/*!
 * \brief load unsigned int value
 */
inline bool load(const std::string& name, size_t& val, bool verbose=false)
{
  int val_;
  if(!load(name, val_, verbose))
  {
    return false;
  }
  if(val_ < 0)
  {
    if(verbose)
    {
      ROS_ERROR("load: could not load '%s'.", name.c_str());
    }
    return false;
  }
  val = val_;
  return true;
}

/*!
 * \brief load scalar value
 */
inline bool load(const std::string& name, cc::Scalar& val, bool verbose=false)
{
  if(!has(name, verbose))
  {
    return false;
  }
  if(!ros::param::get(name, val))
  {
    if(verbose)
    {
      ROS_ERROR("load: could not load '%s'.", name.c_str());
    }
    return false;
  }
  return true;
}

/*!
 * \brief load string value
 */
inline bool load(const std::string& name, std::string& val, bool verbose=false)
{
  if(!has(name, verbose))
  {
    return false;
  }
  if(!ros::param::get(name, val))
  {
    if(verbose)
    {
      ROS_ERROR("load: could not load '%s'.", name.c_str());
    }
    return false;
  }
  return true;
}

/*!
 * \brief load std::vector value.
 * vector size is variable if force_dim=0
 */
inline bool load(
  const std::string& name, 
  std::vector<cc::Scalar>& val, 
  size_t force_dim = 0,
  bool verbose=false)
{
  if(!has(name, verbose))
  {
    return false;
  }
  if(!ros::param::get(name, val))
  {
    if(verbose)
    {
      ROS_ERROR("load: could not load '%s'.", name.c_str());
    }
    return false;
  }
  if(force_dim > 0)
  {
    if(val.size() != force_dim)
    {
      ROS_ERROR("load: '%s' has dim=%ld != desired=%ld", 
        name.c_str(), val.size(), force_dim);
      return false;
    }
  }
  return true;
}

/*!
 * \brief load generic eigen matrix value.
 * If fixed size: loaded dimension must match
 * If variable size and rows and cols specified: used as size
 * Else resized by loaded param.
 */
template<typename _Derived>
inline bool load(
  const std::string& name, 
  _Derived& val,
  typename _Derived::Index rows=Eigen::Dynamic,
  typename _Derived::Index cols=Eigen::Dynamic,
  bool verbose=false)
{
  typedef typename _Derived::Index Index;

  Index Rows, Cols, Size;
  Rows = _Derived::RowsAtCompileTime;
  Cols = _Derived::ColsAtCompileTime;
  Size = _Derived::SizeAtCompileTime;

  std::vector<cc::Scalar> v;
  if(Rows == Eigen::Dynamic || Cols == Eigen::Dynamic)
  {
    if(rows != Eigen::Dynamic && cols != Eigen::Dynamic)
    {
      // desired size specified
      Rows = rows;
      Cols = cols;
      Size = rows*cols;
    }

    // matrix is dynamic, resize to correct dimension
    if(!load(name, v, verbose))
    {
      return false;
    }
    if(Rows != Eigen::Dynamic && Cols != Eigen::Dynamic)
    {
      // desired size given
      val.resize(Rows, Cols);
      if(Size != v.size())
      {
        if(verbose)
        {
          ROS_ERROR("load: '%s' has dim=%ld != desired=%ld", 
            name.c_str(), v.size(), Size);
        }
        return false;
      }
    }
    else if(Rows == Eigen::Dynamic && Cols != Eigen::Dynamic)
    {
      // rows are free
      val.resize(v.size(), Cols);
    }
    else if(Cols == Eigen::Dynamic && Rows != Eigen::Dynamic)
    {
      // cols are free
      val.resize(Rows, v.size());
    }
    else
    {
      // default case
      val.resize(v.size(), 1);
    }
  }
  else
  {
    // matrix is fixed size, check dimension
    if(!load(name, v, Size, verbose))
    {
      return false;
    }
  }

  // store data
  val.setZero();
  for(size_t i = 0; i < val.size(); ++i)
  {
    val(i) = v[i];
  }
  return true;
}

}

#endif // CONTROL_CORE_ROS_H