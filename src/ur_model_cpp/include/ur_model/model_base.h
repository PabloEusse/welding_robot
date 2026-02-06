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

#ifndef MODEL_BASE_H_
#define MODEL_BASE_H_

#include <ros/node_handle.h>
#include <boost/shared_ptr.hpp>

namespace model_interface
{

/**
 * @brief Base class for all models.
 * Gives a generic interface that can be used weather or not 
 * 
 */
class ModelBase
{
public:
    enum State {CONSTRUCTED, INITIALIZED};

public:
    /** 
    * @brief constructor
    *
    * @param controller name
    */
    ModelBase(const std::string& name)
        : name_(name),
          state_(CONSTRUCTED) {}

    virtual ~ModelBase() {} 

    /** 
    * @brief init interal parameter
    *
    * @param ros nh for namespace
    */
    bool initRequest(ros::NodeHandle& nh) {
        if(state_ == INITIALIZED)
            return true;

        if(init(nh)) {
            state_ = INITIALIZED;
            return true;
        }
        return false;
    }

    /**
     * @brief return true if model initialized
     * 
     * @return true 
     * @return false 
     */
    bool isInitialized() const {
        return state_ == INITIALIZED;
    }

    /** 
    * @brief get the controller name
    *
    * @return controller name
    */
    std::string name() const { 
        return name_; 
    }

private:
    /** 
    * @brief init interal parameter
    *
    * @param ros nh for namespace
    */
    virtual bool init(ros::NodeHandle& nh) = 0;

private:
    State state_;
    std::string name_;
};

typedef boost::shared_ptr<model_interface::ModelBase> ModelBasePtr;
typedef boost::shared_ptr<const model_interface::ModelBase> ModelBaseConstPtr;

}

#endif