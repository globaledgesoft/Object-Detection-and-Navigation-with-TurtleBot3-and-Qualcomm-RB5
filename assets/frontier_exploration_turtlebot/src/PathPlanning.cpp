#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <frontier_exploration_turtlebot/CollisionDetector.h>
#include <frontier_exploration_turtlebot/PathPlanning.h>

PathPlanning::PathPlanning() {
  ROS_INFO("Creating the Explorer behaviour...");
  // Set speed parameters
  linearSpeed = 0.1;
  angularSpeed = 0.5;
  // register to publish topic on /cmd_vel
  // to send move velocity commands to the turtlebot
  pubVel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  // creating Subscriber sub  subscribing to scan topic and calling
  // lasercallback function of CollisionDetector class
  sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 50,
                                             &CollisionDetector::laserCallback,
                                             &collisiondetector);

  // Define the initial velocity message
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  count = 0;
  MaxCount = 15;
  // Stop the turtlebot
  pubVel.publish(msg);
}

PathPlanning::~PathPlanning() {
  // Stop the turtlebot before exiting
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  // Stop the turtlebot
  pubVel.publish(msg);
}


void PathPlanning::linearPathGenerator() {
  ros::Rate loop_rate(2);
  //  cheks if the obstacles are in the front of the bot
  if (collisiondetector.checkObstacles() == 1) {
    //  then rotate in CCW
    msg.linear.x = 0;
    msg.angular.z = angularSpeed;
  }
  //  checks if obstacles are at the rear end of the bot
  if (collisiondetector.checkObstacles() == 2) {
    //  then go straight without any angular vel
    msg.linear.x = linearSpeed;
    msg.angular.z = 0.0;
  }
  //  if there are not obstacles present go straight
  if (collisiondetector.checkObstacles() == 0) {
    msg.linear.x = linearSpeed;
    msg.angular.z = 0.0;
  }
  // Publish the cmd message to anyone listening
  pubVel.publish(msg);
  // "Spin" a callback in case we set up any callbacks
  ros::spinOnce();
  // Sleep for the remaining time until we hit our 2 Hz rate
  loop_rate.sleep();
}



void PathPlanning::spiralPathGenerator() {
  // Set up the publisher rate to 2 Hz
  ros::Rate loop_rate(2);
  
  // counter to count the discrete steps to achieve
  // a spiral behaviour
  if (count == MaxCount) {
    count = 1;
  }

  for (int i = 1; i < count; i++) {
    if (i < count) {
      //  cheks if the obstacles are in the front of the bot
      if (collisiondetector.checkObstacles() == 1) {
        //  then rotate in CCW
        msg.linear.x = 0;
        msg.angular.z = angularSpeed;
      }
      //  checks if obstacles are at the rear end of the bot
      if (collisiondetector.checkObstacles() == 2) {
        //  then go straight without any angular vel
        msg.linear.x = linearSpeed;
        msg.angular.z = 0.0;
      }
      //  if there are not obstacles present go straight
      if (collisiondetector.checkObstacles() == 0) {
        msg.linear.x = linearSpeed;
        msg.angular.z = 0.0;
      }
      // Publish the twist message to anyone listening
      pubVel.publish(msg);
      // "Spin" a callback in case we set up any callbacks
      ros::spinOnce();
      // Sleep for the remaining time until we hit our 2 Hz rate
      loop_rate.sleep();

      if (i == count - 1) {
        if (collisiondetector.checkObstacles() == 1) {
          msg.linear.x = 0;
          msg.angular.z = angularSpeed;
        }
        if (collisiondetector.checkObstacles() == 2) {
          msg.linear.x = linearSpeed;
          msg.angular.z = 0.0;
        }
        if (collisiondetector.checkObstacles() == 0) {
          msg.linear.x = 0.0;
          msg.angular.z = angularSpeed;
        }

        // publish the msg to the /cmd_vel topic
        pubVel.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
      }
    }
  }
  count++;
}
