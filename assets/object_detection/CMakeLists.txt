cmake_minimum_required(VERSION 3.5.0)
project(object_detection)

set(PYTHON_EXECUTABLE /usr/bin/python3)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin QUIET)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    message_generation
    sensor_msgs
    cv_bridge
)

add_message_files(
  FILES
  FloatList.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  LIBRARIES object_detection
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

if (catkin_FOUND)

  catkin_package()

  install(
    DIRECTORY msgs
    DESTINATION share/${PROJECT_NAME}
  )

  install(
    DIRECTORY scripts
    DESTINATION share/${PROJECT_NAME}
  )

  install(
    PROGRAMS 
      scripts/rb5_object_detection.py
    DESTINATION lib/${PROJECT_NAME}
  )

  
  install(
    PROGRAMS 
      scripts/MC.py
    DESTINATION lib/${PROJECT_NAME}
  )


  include_directories(
    ${catkin_INCLUDE_DIRS}
  )

else()
  message("catkin not found so skipping this ROS1 package")
endif()

