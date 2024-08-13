cmake_minimum_required(VERSION 3.5.0)
project(move_robot)

set(PYTHON_EXECUTABLE /usr/bin/python3)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin QUIET)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    std_msgs
    message_generation
)

add_message_files(
  FILES
  ObjectFound.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES move_robot
  CATKIN_DEPENDS rospy std_msgs roscpp message_runtime
)

if (catkin_FOUND)

  catkin_package()


  install(
    DIRECTORY scripts
    DESTINATION share/${PROJECT_NAME}
  )


  install(
    PROGRAMS 
      scripts/sample.py
    DESTINATION lib/${PROJECT_NAME}
  )

  install(
    PROGRAMS 
      scripts/Test.py
    DESTINATION lib/${PROJECT_NAME}
  )

  include_directories(
    ${catkin_INCLUDE_DIRS}
  )

else()
  message("catkin not found so skipping this ROS1 package")
endif()

