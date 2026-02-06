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


#ifndef CONTROL_CORE_PARAMETER_H
#define CONTROL_CORE_PARAMETER_H

#include <control_core/common/ros.h>
#include <memory>
#include <unordered_map>

namespace cc
{

/*!
 * \brief The Entry Class
 */
class Entry
{
public:
  std::string name_;
  std::string identifier_;
  bool is_required_;
  bool is_loaded_;

public:
  Entry(const std::string& identifier, bool is_required=true) :
    identifier_(identifier),
    is_required_(is_required),
    is_loaded_(false)
  {
  }

  virtual bool load(const std::string& ns) = 0;
};

/*!
 * \brief The TypedEntry Class
 */
template<typename T>
class TypedEntry : public Entry 
{
public:
  T val_;

public:
  TypedEntry(const std::string& identifier) :
    Entry(identifier, true)
  {
  }

  TypedEntry(const std::string& identifier, const T& default_val, bool is_required=true) :
    Entry(identifier, is_required),
    val_(default_val)
  {
  }

  bool load(const std::string& ns)
  {
    bool verbose = true;
    if(!is_required_)
    {
      verbose = false;
    }
    is_loaded_ = cc::load(ns + identifier_, val_, verbose);
    return is_loaded_;
  }
};

/*!
 * \brief The Parameter Class
 */
class Parameter
{
public:
  typedef std::unordered_map<std::string, std::shared_ptr<Entry> > Entries;
  typedef Entries::iterator EntriesIter;
  typedef Entries::const_iterator CEntriesIter;

protected:
  std::string ns_;
  Entries entries_;
  
public:
  Parameter(const std::string& ns = "") : 
    ns_(prepareNs(ns))
  {
  }

  Parameter(const ros::NodeHandle& nh, const std::string& sub_ns = "") : 
    ns_(prepareNs(nh.getNamespace(), sub_ns))
  {
  }

  std::string nameSpace() const
  {
    return ns_;
  }

  template<typename T>
  bool addOptional(const std::string& name, const T& default_val)
  {
    std::string key = prepareName(name);

    EntriesIter it = entries_.find(key);
    if(it != entries_.end())
    {
      ROS_WARN("Parameter add: key '%s' already exsists, overwriting", name.c_str());
      it->second = std::make_shared<TypedEntry<T> >(key, default_val, false);
    }
    else
    {
      entries_[key] = std::make_shared<TypedEntry<T> >(key, default_val, false);
    }
    return true;
  }

  template<typename T>
  bool addRequired(const std::string& name)
  {
    std::string key = prepareName(name);
    EntriesIter it = entries_.find(key);
    if(it != entries_.end())
    {
      ROS_WARN("Parameter add: key '%s' already exsists, overwriting", name.c_str());
      it->second = std::make_shared<TypedEntry<T> >(key);
    }
    else
    {
      entries_[key] = std::make_shared<TypedEntry<T> >(key);
    }
    return true;
  }

  bool load(const std::string& ns)
  {
    ns_ = prepareNs(ns);
    return load();
  }

  bool load(const ros::NodeHandle& nh, const std::string& sub_ns="")
  {
    ns_ = prepareNs(nh.getNamespace(), sub_ns);
    return load();
  }

  bool load()
  {
    bool ret = true;
    for(EntriesIter it = entries_.begin(); it != entries_.end(); ++it)
    {
      if(it->second->is_required_)
        ret &= it->second->load(ns_);
      else
        it->second->load(ns_);
    }
    return ret;
  }

  template<typename T> 
  bool has(const std::string& name)
  {
    std::string key = prepareName(name);
    CEntriesIter it = entries_.find(key);
    if(it == entries_.end())
    {
      ROS_ERROR("Parameter get: Key '%s' not found.", name.c_str());
      return false;
    }
    return (std::dynamic_pointer_cast<TypedEntry<T> >(it->second) != nullptr);
  }

  template<typename T>
  bool get(const std::string& name, T& val) const
  {
    std::string key = prepareName(name);
    CEntriesIter it = entries_.find(key);
    if(it == entries_.end())
    {
      ROS_ERROR("Parameter get: Key '%s' not found.", name.c_str());
      return false;
    }

    std::shared_ptr<TypedEntry<T> > ptr = 
      std::dynamic_pointer_cast<TypedEntry<T> >(it->second);

    if(ptr)
    {
      if(it->second->is_required_ && !it->second->is_loaded_)
      {
        ROS_ERROR("Parameter get: Requried Key '%s' not loaded.", name.c_str());
        val = ptr->val_;
        return false;
      }
      else
      {
        val = ptr->val_;
        return true;
      }
    }
    ROS_ERROR("Parameter get: Key '%s' found, but wrong template type.", name.c_str());
    return false;
  }

  template<typename T>
  T get(const std::string& name) const
  {
    T val = T();
    get(name, val);
    return val;
  }

  std::string toString()
  {
    std::ostringstream out;

    int i = 0;
    out << "[" << ns_ << "]:" << std::endl;
    for(EntriesIter it = entries_.begin(); it != entries_.end(); ++it)
    {
      out << " - " << it->first << std::endl;
      i++;
    }
    return out.str();
  }

protected:
  std::string prepareNs(const std::string& ns, const std::string& sub_ns = "") const
  {
    std::string ns_mod = ns;
    std::string sub_ns_mod = prepareName(sub_ns);

    if(ns_mod.empty())
      ns_mod.push_back('/');
    if(ns_mod.front() != '/')
      ns_mod.insert(ns_mod.begin(),'/');
    if(ns_mod.back() != '/')
      ns_mod.push_back('/');
    if(!ns.empty())
    {
      ns_mod.append(sub_ns_mod);
      ns_mod.push_back('/');
    }
    return ns_mod;
  }

  std::string prepareName(const std::string& name) const
  {
    std::string name_mod = name;
    if(!name_mod.empty())
    {
      while(name_mod.front() == '/')
        name_mod.erase(name_mod.begin());
      while(name_mod.back() == '/')
        name_mod.pop_back();
    }
    return name_mod;
  }
};

}

#endif // CONTROL_CORE_PARAMETER_H