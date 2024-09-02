

#ifndef INCLUDE_FRONTIER_EXPLORATION_TURTLEBOT_COLLISIONDETECTOR_H_
#define INCLUDE_FRONTIER_EXPLORATION_TURTLEBOT_COLLISIONDETECTOR_H_

#include <sensor_msgs/LaserScan.h>


class CollisionDetector {
 private:
  // variable to detect collision
  int CollisionFlag;

 public:

  CollisionDetector();


  ~CollisionDetector();


  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);


  int checkObstacles();
};

#endif  // INCLUDE_FRONTIER_EXPLORATION_TURTLEBOT_COLLISIONDETECTOR_H_
