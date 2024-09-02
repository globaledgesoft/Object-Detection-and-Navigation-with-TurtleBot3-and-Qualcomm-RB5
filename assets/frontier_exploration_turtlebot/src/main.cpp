#include <ros/ros.h>
#include <std_msgs/String.h>
#include <frontier_exploration_turtlebot/PathPlanning.h>
#include <frontier_exploration_turtlebot/CollisionDetector.h>
#include <iostream>
#include <chrono>

bool detectionReceived = false;  // Flag to indicate if a detection message has been received
std::chrono::time_point<std::chrono::steady_clock> lastDetectionTime;


// Callback function for the /detection topic
void detectionCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("Received detection message: %s", msg->data.c_str());
  detectionReceived = true;  // Set the flag when a detection message is received
  lastDetectionTime = std::chrono::steady_clock::now(); 
}


int main(int argc, char* argv[]) {
  // Initialize the ros node
  ros::init(argc, argv, "frontierExplorer");
  ros::NodeHandle nh;
  // Variable to store user input
  int userChoice;
  std::cout << "Welcome to Turtlebot Explorer" << std::endl;
  std::cout << "Once you are satisfied with the searching" << std::endl;
  std::cout << "Would you like to take linear path (0) or spiral path finder (1)?" " (Enter 0 or 1): ";
  // read the userInput
  std::cin >> userChoice;
  // create the PathPlanning object
  PathPlanning pathPlanning;

  // Subscribe to the /detection topic
  ros::Subscriber detectionSub = nh.subscribe("/detection", 10, detectionCallback);
  ros::Rate loop_rate(10); 

  while (ros::ok()) {
    // Check if detection has been received and if more than 1 seconds have passed since the last detection
    if (detectionReceived) {
      auto now = std::chrono::steady_clock::now();
      auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(now - lastDetectionTime).count();
      if (elapsed_seconds > 2) {
        detectionReceived = false;
      }
    }

    if (!detectionReceived) {
      if (userChoice == 0) {
        pathPlanning.linearPathGenerator();
      } else if (userChoice == 1) {
        pathPlanning.spiralPathGenerator();
      } else {
        // if the user inputs other than 0 or 1 initialize linear behaviour
        ROS_WARN_ONCE("Applying linearPathGenerator by default");
        pathPlanning.linearPathGenerator();
      }
    } else {
      ROS_INFO("Detection received, halting path planning.");
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
