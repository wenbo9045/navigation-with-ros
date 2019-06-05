#ifndef LOCAL_PLANNER_TRI_PLANNER_ROS_H_
#define LOCAL_PLANNER_TRI_PLANNER_ROS_H_

#include <boost/thread.hpp>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include "odometry_helper_ros.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/Vector3.h>
#include <utility>
#include "trajectory.h"
#include <pcl_ros/publisher.h>
#include "map_grid_cost_point.h"
#include "map_grid.h"
#include "rnav/local_map/local_map.h"
#include "rnav/local_planner/goal_functions.h"
#include <string>
#include "Position_float.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <sstream>


//#include <tri_local_planner/circlefitsolver.h>

using namespace std;

namespace tri_local_planner {

  class TRIPlannerROS{
    public:

      TRIPlannerROS();

      ~TRIPlannerROS();
      void initialize(string name,tf::TransformListener* tf,localMap::localMap* costmap);
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel); 
      
      bool triComputeVelocityCommands(tf::Stamped<tf::Pose> &global_pose, geometry_msgs::Twist& cmd_vel);     
       //获取局部代价地图范围内的全局路径transformed_plan
       bool getTransformPlan(tf::Stamped<tf::Pose>& current_pose,const std::vector<geometry_msgs::PoseStamped>& global_plan, std::vector<geometry_msgs::PoseStamped>& transformed_plan);

      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
      void setGoal(geometry_msgs::PoseStamped& goal);

      bool isInitialized() {
        return initialized_;
      }
      
      //(1).终点判断
      bool isPositionReached(tf::Stamped<tf::Pose> global_pose);
      void computeVelocityCommandsStopRotate(geometry_msgs::Twist& cmd_vel);
      bool isGoalReached();
      //订阅目标点
   //   void goal_callback(move_base_msgs::MoveBaseActionGoal);
      double xy_goal_tolerance_;
      double yaw_goal_tolerance_;    
      bool position_reached_check;
      tf::Stamped<tf::Pose> goal_pose;
      geometry_msgs::PoseStamped plan_goal_pose;          
      //(2).角度判断
      bool isOrientationAjusted(tf::Stamped<tf::Pose> global_pose);
      double orientation_tolerance;
      double start_orientation;           
      //(3).起始位置判断
      bool isBegin(tf::Stamped<tf::Pose> global_pose);
      double start_pose_x,start_pose_y;
      bool updateStartPose;   
      bool ifupdated;       
      //(4).终点位置判断  
      bool isEnd(tf::Stamped<tf::Pose> global_pose);
      //(5).转弯判断
      bool circleLeastFit(const std::vector<geometry_msgs::PoseStamped>& new_plan, double &center_x, double &center_y, double &radius);
      bool getLocalPlanCheckingPoint();
      ros::Publisher marker_point_pub;
      //CircleFitSolver circle;
      //(6).直线判断
      bool isPlanStraight(tf::Stamped<tf::Pose> global_pose, const std::vector<geometry_msgs::PoseStamped>& new_plan);
      double get_orientation(tf::Stamped<tf::Pose> global_pose,double local_goal_x,double local_goal_y);
      int maxPoints(vector<Position_float> &points);
      Position_float Position_temp;
      vector<Position_float> buff;
      //(7).安全判断
      bool isRobotSafe(tf::Stamped<tf::Pose> global_pose);
      void probeGenerator(tf::Stamped<tf::Pose> global_pose);
      std::vector<double>  checkProbeCost; 
      std::vector<geometry_msgs::Pose> checkProbe;
      int recovery_counter;
      int recovery_counter_add; 
      ros::Publisher probe_pub;  
      visualization_msgs::MarkerArray probe_markers;
         
      nav_msgs::Path gl_plan_;
      std::vector<geometry_msgs::PoseStamped> gl_plan_vector;    
      std::vector<geometry_msgs::PoseStamped> transformed_plan;   
  
      //调速参数
      double vel_set_;
      double w_set;
      void vel_select(tf::Stamped<tf::Pose> &global_pose,double& vel_set);
      void adjust_vel(double& vel_set,double min_vel,double max_vel,bool increase);
      void adjust_w(double& w_set,double min_w,double max_w,bool increase);
      int count_;
      double max_turn_radius;
      int min_straight_points;
      double straight_orientation_tolerance;
      double diff_begin_tolerance;
      double diff_end_tolerance;
      string state = "";
      string pre_state = "";
      Trajectory traj_temp;
    
      //1.速度采样
      void vel_sample(tf::Stamped<tf::Pose> robot_vel,vector<pair<double,double> >& vsamples_,double vel_set);
      //速度采样使用参数
      double max_vel_th;
      double min_vel_th;
      geometry_msgs::Vector3 acc_lim;
      double acc_lim_x;
      double acc_lim_y;
      double acc_lim_theta;

      double min_vel_x;
      double max_vel_x;
      double min_vel_y;
      double max_vel_y;
  
      double max_trans_vel;
      double min_trans_vel;
      double sim_time_;  
      double vx_samples;
      double vth_samples;  
  
      double sim_granularity,angular_sim_granularity;
      string frame_id;
  
      //2.轨迹生成
      void paths_generate(vector<pair<double,double> > vel_samples_,vector<Trajectory>& all_explored);   
      pcl::PointCloud<MapGridCostPoint>* traj_cloud_;
      pcl_ros::Publisher<MapGridCostPoint> traj_cloud_pub_; 
      bool publish_traj_pc_;
  
  
      //3.轨迹评分
      void score_paths(vector<Trajectory> all_explored,Trajectory& result_traj);
      double path_distance_bias,goal_distance_bias,occdist_scale;
      double pdist_scale_, gdist_scale_, occdist_scale_;
      ros::Publisher best_traj_pub;
      //地图预处理
      MapGrid goal_prepare_map,path_prepare_map;
  
     double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
     {
       return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
     }
     
    bool checkAngle(geometry_msgs::PoseStamped firstPoint,
                    geometry_msgs::PoseStamped secondPoint,
                    geometry_msgs::PoseStamped thirdPoint)
    {
     double firstLineDist,secondLineDist,thirdLineDist;
     firstLineDist = distance(firstPoint,secondPoint);
     secondLineDist = distance(secondPoint,thirdPoint);
     thirdLineDist = distance(firstPoint,thirdPoint);
     double angleMin =  160*3.14159/180;
     double angleNow = acos((firstLineDist*firstLineDist+
      			     secondLineDist*secondLineDist-
   			     thirdLineDist*thirdLineDist)/(2*firstLineDist*secondLineDist));

     if(angleNow < angleMin) 
     {
      ROS_ERROR("angleDelta: %f, 1_x: %f, 1_y: %f, 2_x. %f, 2_y.%f 3_x. %f, 3_y. %f",
                 (angleNow-angleMin)*180/3.14159,
                 firstPoint.pose.position.x,firstPoint.pose.position.y,
                 secondPoint.pose.position.x,secondPoint.pose.position.y,
                 thirdPoint.pose.position.x,thirdPoint.pose.position.y);
       return false;
     }
     return true;
    }
    
    bool checkGoalSet(std::vector<geometry_msgs::PoseStamped>& transientList,geometry_msgs::PoseStamped& selectedPose)
    {
     if(transientList.size() < 3)
     return true;
     else
     {
      geometry_msgs::PoseStamped firstPoint = transientList[0];
      geometry_msgs::PoseStamped secondPoint = transientList[1];
      geometry_msgs::PoseStamped thirdPoint;
      for(int num = 2; num < transientList.size(); num++)
      {
       thirdPoint = transientList[num];
       if(!checkAngle(firstPoint,secondPoint,thirdPoint)) 
       {
        selectedPose = secondPoint;
        return false;
       }
       firstPoint = secondPoint;
       secondPoint = thirdPoint;
      }
     }
     return true;
    }
    
    bool checkLocalPlan(std::vector<geometry_msgs::PoseStamped>& local_plan)
    {
     geometry_msgs::PoseStamped position_pre = transformed_plan[0];
     vector<geometry_msgs::PoseStamped> position_set;
     float dist;
     position_set.push_back(transformed_plan[0]);
     for(int counter = 0; counter < transformed_plan.size()/2; ++counter)
     {
       dist = distance(position_pre,transformed_plan[counter]);
       if(dist>=0.3) 
       {
         position_set.push_back(transformed_plan[counter]);
         position_pre = transformed_plan[counter];
       }
     }
     unsigned int mx,my;
     for(int i = 0; i < position_set.size(); ++i)
     {
      costmap_->worldToMap(position_set[i].pose.position.x, position_set[i].pose.position.y, mx, my);
      if(costmap_->getCost(mx,my)> 240) return false;
     }
     return true;
    }
    
    
    private:
      ros::Subscriber goal_sub_;
      ros::NodeHandle nn;

      tf::TransformListener* tf_; 
      ros::Publisher g_plan_pub_, l_plan_pub_;

      localMap::localMap* costmap_;

      bool setup_;

      tf::Stamped<tf::Pose> current_pose_;

      bool initialized_;

      tri_local_planner::OdometryHelperRos odom_helper_;
      std::string odom_topic_;
      
      
      bool receiveTraj;
      
      visualization_msgs::Marker point_set;
      ros::Publisher angle_deviation_pub;
      int stop_times_ = 0;
      
      int stop_rotate_dir = 0;
  };
};
#endif
