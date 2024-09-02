
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <frontier_exploration_turtlebot/CollisionDetector.h>
#include <vector>
#include <algorithm>

CollisionDetector::CollisionDetector() {
  // print out init message to the screen
  ROS_INFO("Initializing Collision Detection!");
  CollisionFlag = 0;
}

CollisionDetector::~CollisionDetector() {
}

void CollisionDetector::laserCallback(
    const sensor_msgs::LaserScan::ConstPtr& msg) {
  //  stores the LaserScan range data in vector
  std::vector<float> msgstore = msg->ranges;
  int countInd = 0;

  // read all the members in the msgstore vector which contains range
  // messages from the /scan topic
  for (auto i : msgstore) {
    int a = std::isinf(i);
    if (a == 1) {
      // replace all the inf values to 0.5
      msgstore[countInd] = 0.5;
    }
    countInd++;
  }

  // variable to stores and sums up quadrant data
  double sumq1 = 0, sumq2 = 0;
  int counter = 0;

  for (auto i : msgstore) {
    // only inspect values that are above zero and below 0.5m
    // this will create a buble of radius 0.5m
    // objects outside the radius are ignored
    if (i >= 0 && i <= 0.5) {
      // targeting angles from 0 to 45 deg in quadarant 1
      if (counter >= 10 && counter < 50) {
        sumq1 = sumq1 + msgstore[counter];
      }
      // targeting angles from 315 to 359 in quadrant 1 (blind spot)
      if (counter >= 310 && i <= 350) {
        sumq1 = sumq1 + msgstore[counter];
      }
      // targeting angles from 135 to 225 in quadrant 2
      if (counter >= 165 && counter < 195) {
        sumq2 = sumq2 + msgstore[counter];
      }
    }
    counter++;
  }
  ROS_INFO_STREAM("Front area: " << sumq1);
  ROS_INFO_STREAM("Rear area: " << sumq2);

  // calculates the quadrant at which the distance to the obstacle is minimum
  double minelement = std::min(sumq1, sumq2);
  if (minelement == sumq1) {
    // if minimum is in quadrant 1 then collisionflag is set to 1
    CollisionFlag = 2;
  }
  if (minelement == sumq2) {
    // if minimum is in quadrant 2 then collisionflag is set to 2
    CollisionFlag = 1;
  }
  if (sumq1 == sumq2) {
    // if minimum is in quadrant 1 then collisionflag is set to 1
    CollisionFlag = 0;
  }
}

int CollisionDetector::checkObstacles() {
  return CollisionFlag;
}
