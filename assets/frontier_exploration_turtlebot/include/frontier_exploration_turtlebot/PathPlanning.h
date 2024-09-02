

#ifndef INCLUDE_FRONTIER_EXPLORATION_TURTLEBOT_PATHPLANNING_H_
#define INCLUDE_FRONTIER_EXPLORATION_TURTLEBOT_PATHPLANNING_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <frontier_exploration_turtlebot/CollisionDetector.h>

class PathPlanning {
 private:
  // Create CollisionDetector Object
  CollisionDetector collisiondetector;
  // variable to generate discrete spiral steps
  int count;
  // variable to declare maximum spiral step count
  int MaxCount;
  // variable to set the linear speed
  float linearSpeed;
  // variable to set the angular speed
  float angularSpeed;
  // declare a variable for velocities
  geometry_msgs::Twist msg;
  // publishes velocity
  ros::Publisher pubVel;
  // node handler
  ros::NodeHandle nh;
  // subscriber
  ros::Subscriber sub;

 public:

  PathPlanning();


  ~PathPlanning();

 
  void spiralPathGenerator();


  void linearPathGenerator();



};

#endif  // INCLUDE_FRONTIER_EXPLORATION_TURTLEBOT_PATHPLANNING_H_
