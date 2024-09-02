
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <gtest/gtest.h>
#include <frontier_exploration_turtlebot/CollisionDetector.h>
#include <frontier_exploration_turtlebot/PathPlanning.h>



// global variable for testing
float linX = 0.0, angZ;


void testCallback(const geometry_msgs::Twist msg) {
  linX = msg.linear.x;
  angZ = msg.angular.z;
}


TEST(PathPlanningTest, InitializationErrorTest) {
  ros::NodeHandle nh;
  EXPECT_NO_FATAL_FAILURE(PathPlanning pathPlanner);
}


TEST(PathPlanningTest, spiralPathGeneratorTest) {
  // created NodeHandle nh
  ros::NodeHandle nh;
  // creating Publisher testpub to publish laser msg to sca topic
  ros::Publisher testPub = nh.advertise<sensor_msgs::LaserScan>("/scan", 50);
  //  creating  a laserData msg tpe
  sensor_msgs::LaserScan laserData;
  laserData.angle_min = -0.52;
  laserData.angle_max = 0.52;
  laserData.angle_increment = 0.0016;
  laserData.time_increment = 0.0;
  laserData.range_min = 0.44;
  laserData.range_max = 3.0;
  laserData.ranges.resize(50);
  laserData.intensities.resize(50);

  // initialize all the range of laserData to zero
  for (auto& i : laserData.ranges) {
    i = 0.0;
  }

  PathPlanning planner;
  // creating Subscriber testsub  subscribing to scan topic
  // and calling lasercallback function of CollisionDetector class
  ros::Subscriber testSub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 50,
                                                               testCallback);

  int counter = 0;

  while (ros::ok()) {
    // publish the laserData
    testPub.publish(laserData);
    // activate the spiral generator method
    planner.spiralPathGenerator();
    // break the loop after three iterations
    if (counter == 3) {
      break;
    }
    ros::spinOnce();
    counter++;
  }
  // check to see if the linear velocity is what we expect it to be for
  // the laser data
  EXPECT_NEAR(0.2, linX, 0.1);
  EXPECT_EQ(0.0, angZ);
}


TEST(PathPlanningTest, linearPathGeneratorTest) {
  ros::NodeHandle nh;
  // create a publisher to publish the laserscan topic
  ros::Publisher testPub = nh.advertise<sensor_msgs::LaserScan>("/scan", 50);
  //  create a laserData msg for testing purpose
  sensor_msgs::LaserScan laserData;
  laserData.angle_min = -0.52;
  laserData.angle_max = 0.52;
  laserData.angle_increment = 0.0016;
  laserData.time_increment = 0.0;
  laserData.range_min = 0.44;
  laserData.range_max = 3.0;
  laserData.ranges.resize(50);
  laserData.intensities.resize(50);

  // fill range vector to zeros
  for (auto& i : laserData.ranges) {
    i = 0.0;
  }

  // initializing planner object
  PathPlanning planner;
  // creating Subscriber testsub  subscribing to cmd_vel topic
  // and calling lasercallback function of CollisionDetector class
  ros::Subscriber testSub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 50,
                                                               testCallback);

  int counter = 0;

  while (ros::ok()) {
    // publish the test laser data
    testPub.publish(laserData);
    planner.linearPathGenerator();

    // break the loop after three iterations
    if (counter == 3) {
      break;
    }
    ros::spinOnce();
    counter++;
  }
  // check to see if the linear velocity is what we expect it to be for
  // the laser data
  EXPECT_NEAR(0.2, linX, 0.1);
  EXPECT_EQ(0.0, angZ);
}
