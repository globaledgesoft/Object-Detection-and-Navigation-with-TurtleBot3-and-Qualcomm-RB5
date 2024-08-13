cmake_minimum_required(VERSION 3.0.2)
project(whisper_pkg)

set(PYTHON_EXECUTABLE /usr/bin/python3.8)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

catkin_package(
  LIBRARIES whisper_pkg
  CATKIN_DEPENDS rospy std_msgs roscpp  message_runtime
)


if (catkin_FOUND)

  catkin_package()


  install(
    DIRECTORY scripts
    DESTINATION share/${PROJECT_NAME}
  )

  install(
    PROGRAMS 
      scripts/whisper_speech_recognize.py
    DESTINATION lib/${PROJECT_NAME}
  )

  include_directories(
    ${catkin_INCLUDE_DIRS}
  )

else()
  message("catkin not found so skipping this ROS1 package")
endif()



