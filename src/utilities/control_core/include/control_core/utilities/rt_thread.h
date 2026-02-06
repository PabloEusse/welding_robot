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

#ifndef CONTROL_CORE_RT_RTThread_H_
#define CONTROL_CORE_RT_RTThread_H_

#include <control_core/utilities/type_not_assignable.h>

#include <pthread.h>
#include <sys/mman.h>
#include <stdio.h>
#include <malloc.h>
#include <sys/resource.h>
#include <limits.h>

/*! \file RTThread.h
 *  \brief Contains RTThread Wrapper interface
 */

#include <iostream>

namespace cc
{

  /**
   * @brief The RTThread class wrappes a std::Thread object.
   * 
   * This Class can be used as a base class to execute code in another Thread.
   * After the start() is called, the run() function will be executed in
   * a new Thread.
   * 
   * @note This requires sudo right
   */

  class RTThread
  {
    OW_TYPE_NOT_ASSIGNABLE(RTThread)

  protected:
    int priority_;
    int stack_size_;
    int stack_pre_fault_size_;
    bool started_;

    pthread_t pthread;
    pthread_attr_t pthread_attr;

  public:
    /**
     * @brief Construct a new RTThread object
     * 
     * @param priority value between 0 and 99 
     * @param stack_size in bytes
     * @param stack_pre_fault_size in bytes for pre allocation
     * 
     */
    RTThread(int priority = 49, int stack_size = 8*1024, int stack_pre_fault_size = 8*1024) :
      priority_(priority),
      stack_size_(stack_size),
      stack_pre_fault_size_(stack_pre_fault_size),
      started_(false)
    {
    }

    /**
     * @brief Destroy the RTThread object
     * 
     */
    virtual ~RTThread() 
    {
      pthread_cancel(pthread);
    }

    /**
     * @brief checks if the RTThread is stopped
     * 
     * @return true 
     * @return false 
     */
    bool isFinished()
    {
      return !started_;
    }

    /**
     * @brief checks if the RTThread is running
     * 
     * @return true 
     * @return false 
     */
    bool isRunning()
    {
      return started_;
    }

    /**
     * @brief start the RTThread
     * 
     * @return true 
     * @return false 
     */
    bool start() 
    {
      int ret = 0;

      // allready running
      if( started_ )
      {
        return false;
      }

      // init attr
      if( pthread_attr_init(&pthread_attr) )
      {
        fprintf(stderr, "RTThread::start(): error at pthread_attr_init\n");
        return false;
      }

      // set stack size
      printf("stack_size = %d\n", stack_size_);
      if( ret = pthread_attr_setstacksize(&pthread_attr, PTHREAD_STACK_MIN + stack_size_) ) 
      {
        fprintf(stderr, "RTThread::start(): error at pthread_attr_setstacksize: %d\n", ret);
        return false;
      }

      // set sched, prio
      if( pthread_attr_setschedpolicy(&pthread_attr, SCHED_FIFO))
      {
        fprintf(stderr, "RTThread::start(): error at pthread_attr_setschedpolicy\n");
        return false;
      }

      struct sched_param param;
      param.sched_priority = priority_;
      if( pthread_attr_setschedparam(&pthread_attr, &param) )
      {
        fprintf(stderr, "RTThread::start(): error at pthread_attr_setschedparam\n");
        return false;
      }

      // lock mem
      if (mlockall(MCL_CURRENT | MCL_FUTURE))
      {
        fprintf(stderr, "RTThread::start(): error at mlockall\n");
        return false;
      }

      // create new thread, call run fnc
      if( pthread_create(&pthread, &pthread_attr, pthread_member_wrapper, this) ) 
      {
        fprintf(stderr, "RTThread::start(): error creating pthread\n");
        return false;
      }
      return true;   
    }

  private:
    virtual void runInternal()
    {
      started_ = true;
      run();
      started_ = false;
    }

  protected:
    /*!
    * @brief The RTThread execution function.
    */
    virtual void run() = 0;

  private:
      static void* pthread_member_wrapper(void* this_ptr) 
      {
        RTThread* obj = static_cast<RTThread*>(this_ptr);
        obj->runInternal();
      }
  };

} // namespace cc

#endif // CONTROL_CORE_RTThread_H_