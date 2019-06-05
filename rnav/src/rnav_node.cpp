/*
 * A 
 * Yuan rupeng's
 * algorithm to autonomously navigate a robot
 */
#include "ros/ros.h"
#include "rnav/move_base/move_base.h"
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rnav_node");
  ROS_INFO("Start to navigate the robot!");
  tf::TransformListener tf(ros::Duration(10));
  move_base::MoveBase move_base(tf);
  ros::spin();
  return 0;
}
