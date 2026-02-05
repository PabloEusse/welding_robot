execute_process(COMMAND "/home/pablo/Desktop/welding_robot/simulation/build/ur_robot/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/pablo/Desktop/welding_robot/simulation/build/ur_robot/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
