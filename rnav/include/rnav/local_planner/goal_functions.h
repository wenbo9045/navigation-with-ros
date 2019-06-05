#ifndef LOCAL_LOCAL_PLANNER_GOAL_FUNCTIONS_H_
#define LOCAL_LOCAL_PLANNER_GOAL_FUNCTIONS_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>

#include <string>
#include <cmath>

#include <angles/angles.h>
//#include <costmap_2d/costmap_2d.h>
#include "map_grid.h"
namespace tri_local_planner {


  /**
   * @brief  return angle difference to goal to check if the goal orientation has been achieved
   * @param  global_pose The pose of the robot in the global frame
   * @param  goal_x The desired x value for the goal
   * @param  goal_y The desired y value for the goal
   * @return angular difference
   */
  double getGoalOrientationAngleDifference(const tf::Stamped<tf::Pose>& global_pose, double goal_th);

  /**
   * @brief  Publish a plan for visualization purposes
   * @param  path The plan to publish
   * @param  pub The published to use
   * @param  r,g,b,a The color and alpha value to use when publishing
   */
  void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub);

  /**
   * @brief  Trim off parts of the global plan that are far enough behind the robot
   * @param global_pose The pose of the robot in the global frame
   * @param plan The plan to be pruned
   * @param global_plan The plan to be pruned in the frame of the planner
   */
  void prunePlan(const tf::Stamped<tf::Pose>& global_pose, std::vector<geometry_msgs::PoseStamped>& plan, std::vector<geometry_msgs::PoseStamped>& global_plan);
  void transformPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan,std::vector<geometry_msgs::PoseStamped>& transformed_plan);
  double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }

   void transformGoalPose(
      const tf::TransformListener& tf,
      const geometry_msgs::PoseStamped& plan_goal_pose,
      tf::Stamped<tf::Pose>& goal_pose,
      const std::string& global_frame);

};
#endif
