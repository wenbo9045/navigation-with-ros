#ifndef MOVE_BASE_H_
#define MOVE_BASE_H_

#include <vector>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>

#include <std_srvs/Empty.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include "std_msgs/Int32.h"
#include <nav_msgs/OccupancyGrid.h>
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/tf.h"
#include "rnav/nav_map/nav_map.h"
#include "rnav/local_map/local_map.h"
#include "rnav/local_planner/tri_planner_ros.h"
#include "rnav/global_planner/navfn_ros.h"
namespace move_base {

  enum MoveBaseState {
    PLANNING,
    CONTROLLING,
    ADJUSTING, 
    SWITCHING
  };
  
  class MoveBase {
    public:

      MoveBase(tf::TransformListener& tf);
      ~MoveBase();
      bool executeCycle();

    private:
      bool generateRepeatedPlan(std::vector<geometry_msgs::PoseStamped>& global_plan);//用于导航规划
      void pruneGlobalPlan(std::vector<geometry_msgs::PoseStamped>& plan);//剪切全局路径，判断是否需要重新规划
      bool checkGlobalPlan(geometry_msgs::PoseStamped start, const std::vector<geometry_msgs::PoseStamped>& global_plan);
      void publishZeroVelocity();
      void resetState();
      
      void goalReceived(const geometry_msgs::PoseStamped& goal);

      void planThread();
      void navThread();
      double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);
      double angle(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);
      void wakePlanner(const ros::TimerEvent& event);
      
      bool ifDirectlyToGoal(const geometry_msgs::PoseStamped start);
      tf::TransformListener& tf_;

      tri_local_planner::TRIPlannerROS* tc_;
      navfn::NavfnROS* planner_;
      std::string robot_base_frame_, global_frame_;

      tf::Stamped<tf::Pose> global_pose_;
      double planner_frequency_, controller_frequency_,rotation_radius_,cost_threshold_;
      double planner_patience_, controller_patience_;
      ros::Publisher vel_pub_, path_pub_;
      ros::Subscriber goal_sub_;
      ros::NodeHandle nn;

      MoveBaseState state_;

      ros::Time last_valid_plan_, last_valid_control_;

      //set up plan triple buffer
      std::vector<geometry_msgs::PoseStamped> global_plan_;
      
      int round_;
      ros::Timer timer;
      
      std::vector<geometry_msgs::PoseStamped>* latest_plan_;

      //set up the planner's thread
      boost::mutex planner_mutex_;
      boost::condition_variable planner_cond_;
      boost::thread* planner_thread_;
      boost::thread* nav_thread_;

      boost::recursive_mutex configuration_mutex_;
      bool goodToGo;
      bool safetyCheck;
      bool distCheck;
      
      nav_msgs::OccupancyGrid navMap;
      
      navMap::navMap* planner_costmap_ros_;
      localMap::localMap* controller_costmap_ros_;
      bool gotPlan = false;//判断是否得到了规划，如果得到了就用prune的方式不断截短，否则进行规划
      bool goalupdated =false;
      geometry_msgs::PoseStamped goal_;
      
      
      visualization_msgs::Marker checking_set;
      ros::Publisher marker_point_pub;
      
  };
};
#endif

