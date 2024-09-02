#include <ros/ros.h>
#include <gtest/gtest.h>


int main(int argc, char** argv) {
  std::cout<<"GANESH"<<std::endl;
  ros::init(argc, argv, "frontierExplorerTest");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
