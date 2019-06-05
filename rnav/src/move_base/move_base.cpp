#include "rnav/move_base/move_base.h"
#include <cmath>
#include <assert.h>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
static bool ifGo = false;
static bool gotValidPlan = false;
namespace move_base {
  MoveBase::MoveBase(tf::TransformListener& tf) :
    tf_(tf),
    safetyCheck(false),distCheck(false),
    goodToGo(false) {

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;
    private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
    private_nh.param("global_costmap/global_frame", global_frame_, std::string("/map"));
    private_nh.param("controller_frequency", controller_frequency_, 10.0);

    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    goal_sub_ = nh.subscribe("/move_base_simple/goal",1,&MoveBase::goalReceived,this);
    path_pub_ = nh.advertise<nav_msgs::Path>("/move_base/NavfnROS/plan",1);
    //set up the planner's thread
    planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));
    nav_thread_ = new boost::thread(boost::bind(&MoveBase::navThread, this));
    timer = nh.createTimer(20, &MoveBase::wakePlanner, this);
    planner_costmap_ros_ = new navMap::navMap();
    controller_costmap_ros_ = new localMap::localMap();
    tc_ = new tri_local_planner::TRIPlannerROS();
    tc_ -> initialize("base_local_planner",&tf_,controller_costmap_ros_);
    planner_ = new navfn::NavfnROS();
    planner_->initialize("base_global_planner", planner_costmap_ros_);
    //initially, we'll need to make a plan
    state_ = PLANNING;
    ROS_WARN("MOVE BASE CONSTRUCTED");
    
    checking_set.header.frame_id = "/map";
    checking_set.type = visualization_msgs::Marker::POINTS;
    checking_set.scale.x = 0.1;
    checking_set.scale.y = 0.1;
    checking_set.color.g = 1.0;
    checking_set.color.a = 1.0;
    marker_point_pub = nh.advertise<visualization_msgs::Marker>("visualize_chekcing_points",10);
  }
  
  void MoveBase::goalReceived(const geometry_msgs::PoseStamped& goal)
  {
   goalupdated = true;
   goodToGo = true;
   goal_ = goal;
   tc_->setGoal(goal_);
    /*std::vector<geometry_msgs::PoseStamped> global_plan;
    tf::Stamped<tf::Pose> global_pose;
    global_pose.setIdentity();
    tf::Stamped < tf::Pose > robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = "base_link";
    robot_pose.stamp_ = ros::Time();
     // get the global pose of the robot
     try
     {
      tf_.transformPose("map", robot_pose, global_pose);
     }
     catch (tf::LookupException& ex)
     {
      ROS_ERROR("Cannot transform from map to robot");
      return;
     }
    geometry_msgs::PoseStamped start;
    tf::poseStampedTFToMsg(global_pose, start);
   planner_->makePlan(start, goal, global_plan); 
   nav_msgs::Path p;
   p.header.stamp =ros::Time::now();
   p.header.frame_id = "/map";
   for(int i = 0; i<global_plan.size();++i)
   {
    p.poses.push_back(global_plan[i]);
   }
   path_pub_.publish(p);*/
  }


  MoveBase::~MoveBase(){
    delete planner_costmap_ros_;
    delete controller_costmap_ros_;
    planner_thread_->interrupt();
    planner_thread_->join();
    delete planner_thread_;
    delete nav_thread_;
    delete tc_;
  }
  
  bool MoveBase::checkGlobalPlan(geometry_msgs::PoseStamped start, const std::vector<geometry_msgs::PoseStamped>& global_plan)
  {
   if(distance(start,goal_) <= 3) return true;
   geometry_msgs::PoseStamped check_position;
   int count;
   for(count = 0; count < global_plan.size(); ++count)
   {
    if(distance(start,global_plan[count]) >= 2) {check_position = global_plan[count]; break;}
   }
   
   unsigned int mx,my;   
   for(int i = 0; i < 5; ++i)
   {
    planner_costmap_ros_->worldToMap(check_position.pose.position.x, check_position.pose.position.y, mx, my);
    if(planner_costmap_ros_->getCost(mx,my) > 200) 
    return false;
    
    while(count < global_plan.size())
    {
     if(distance(check_position,global_plan[count]) >= 0.3) {check_position = global_plan[count];break;}
     count++;
    }
   }
   return true;
  }
  
  void MoveBase::pruneGlobalPlan(std::vector<geometry_msgs::PoseStamped>& global_plan)
  {
    //ROS_WARN("PRUNE PLAN");
    tf::Stamped<tf::Pose> global_pose;
    global_pose.setIdentity();
    tf::Stamped < tf::Pose > robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = "base_link";
    robot_pose.stamp_ = ros::Time();
     // get the global pose of the robot
     try
     {
      tf_.transformPose("map", robot_pose, global_pose);
     }
     catch (tf::LookupException& ex)
     {
      ROS_ERROR("Cannot transform from map to robot");
      return;
     }
    geometry_msgs::PoseStamped start;
    tf::poseStampedTFToMsg(global_pose, start);
   std::vector<geometry_msgs::PoseStamped> new_plan;
   if(ifDirectlyToGoal(start))
   {
    gotPlan = true;
    float dist = distance(start,goal_);
    float delta_x = (goal_.pose.position.x - start.pose.position.x)/dist*0.02;
    float delta_y = (goal_.pose.position.y - start.pose.position.y)/dist*0.02;
    geometry_msgs::PoseStamped path_point = start;
    new_plan.push_back(path_point);
    while(distance(path_point,goal_) > 0.04)
    {
     path_point.pose.position.x += delta_x;
     path_point.pose.position.y += delta_y;
     new_plan.push_back(path_point);
    }
    global_plan = new_plan;
   }
   /////////////////////////////////////////////////////////////////当规划的路径的第一个点与当前位置相差大于0.3m时，重新规划路径。
    std::vector<geometry_msgs::PoseStamped>::iterator iter = global_plan.begin();
    float dist,ang;
    dist = distance(*iter,start);
    if(dist >= 0.3)
    {
     planner_->makePlan(start, goal_, global_plan); 
    }
    else
    {
     int counter = 0;
     
     for(auto iter = global_plan.begin(); iter != global_plan.end(); iter++)
     {
      counter++;
      dist = distance(*iter,start);
      ang = angle(*iter,start);
      ////////////////////////////////////////////////这没看懂在干吗！！！,是不是应该大于0.2
      if(dist < 0.2 || counter == 10) break;
      else global_plan.erase(iter);
     }
    }

   nav_msgs::Path p;
   p.header.stamp =ros::Time::now();
   p.header.frame_id = "/map";
   for(int i = 0; i<global_plan.size();++i)
   {
    p.poses.push_back(global_plan[i]);
   }
   path_pub_.publish(p);
   tc_->setPlan(global_plan);
  }
  
  //用于路径规划
  bool MoveBase::generateRepeatedPlan(std::vector<geometry_msgs::PoseStamped>& global_plan)
  {
  std::vector<geometry_msgs::PoseStamped> new_plan;
    ROS_WARN("MAKING PLAN");
    tf::Stamped<tf::Pose> global_pose;
    global_pose.setIdentity();
    tf::Stamped < tf::Pose > robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = "base_link";
    robot_pose.stamp_ = ros::Time();
     // get the global pose of the robot
     try
     {
      tf_.transformPose("map", robot_pose, global_pose);
     }
     catch (tf::LookupException& ex)
     {
      ROS_ERROR("Cannot transform from map to robot");
      return false;
     }
    geometry_msgs::PoseStamped start;
    tf::poseStampedTFToMsg(global_pose, start);
    
   if(ifDirectlyToGoal(start))
   {
    gotPlan = true;
    float dist = distance(start,goal_);
    float delta_x = (goal_.pose.position.x - start.pose.position.x)/dist*0.02;
    float delta_y = (goal_.pose.position.y - start.pose.position.y)/dist*0.02;
    geometry_msgs::PoseStamped path_point = start;
    new_plan.push_back(path_point);
    while(distance(path_point,goal_) > 0.04)
    {
     path_point.pose.position.x += delta_x;
     path_point.pose.position.y += delta_y;
     new_plan.push_back(path_point);
    }
    global_plan = new_plan;
   }
   else
    gotPlan = planner_->makePlan(start, goal_, global_plan); 
   nav_msgs::Path p;
   p.header.stamp =ros::Time::now();
   p.header.frame_id = "/map";
   for(int i = 0; i<global_plan.size();++i)
   {
    p.poses.push_back(global_plan[i]);
   }
   path_pub_.publish(p);
   tc_->setPlan(global_plan);
   ifGo = true;
   goodToGo = true;
   //gotPlan = true;
   return true;
  }

  
  void MoveBase::publishZeroVelocity(){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
  }

  void MoveBase::wakePlanner(const ros::TimerEvent& event)
  {
    // we have slept long enough for rate
    planner_cond_.notify_one();
  }
  
  void MoveBase::navThread()
  {
    ros::Rate r(controller_frequency_);
    ros::NodeHandle n;
    while(n.ok())
    {
     r.sleep();
     if(goodToGo)
     {
      ros::WallTime start = ros::WallTime::now();
      //the real work on pursuing a goal is done here
      bool done = executeCycle();
      //if we're done, then we'll return from execute
      if(done)
        goodToGo = false;
      //check if execution of the goal has completed in some way
      ros::WallDuration t_diff = ros::WallTime::now() - start;
      ROS_DEBUG_NAMED("move_base","Full control cycle time: %.9f\n", t_diff.toSec());

      //make sure to sleep for the remainder of our cycle time
      if(r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
      ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
     }
    }
    return;
  }

  
  void MoveBase::planThread(){
    ROS_DEBUG_NAMED("move_base_plan_thread","Starting planner thread...");
    ros::NodeHandle n;
 
    bool wait_for_wake = false;
    boost::unique_lock<boost::mutex> lock(planner_mutex_);
    while(n.ok()){
      planner_cond_.wait(lock);
      ros::Time start_time = ros::Time::now();
      if(!goalupdated||!goodToGo) continue;
      lock.unlock();
     // ROS_WARN("move_base_plan_thread");

      if(!gotPlan) generateRepeatedPlan(global_plan_);
      else pruneGlobalPlan(global_plan_);


      lock.lock();
      state_ = CONTROLLING;
      lock.unlock();
      lock.lock();
    }
  }
  
  double MoveBase::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }
  
  double MoveBase::angle(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
   double angle;
   angle = atan2(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }
  
  
    bool MoveBase::executeCycle(){
    boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
    //we need to be able to publish velocity commands
    geometry_msgs::Twist cmd_vel;

    switch(state_){
      //if we are in a planning state, then we'll attempt to make a plan
      case PLANNING:
         ROS_INFO("case:PLANNING");
         {
          boost::mutex::scoped_lock lock(planner_mutex_);
          planner_cond_.notify_one();  
         }  

        ROS_DEBUG_NAMED("move_base","Waiting for plan, in the planning state.");
        break;

      //if we're controlling, we'll attempt to find valid velocity commands
      case CONTROLLING:
         // ROS_WARN("case:CONTROLLING");
        if(tc_->isGoalReached()){
          ROS_INFO("move_base Goal reached!");
          goodToGo = false;
          ifGo = false;
          gotPlan = false;
          resetState();
          return true;
        }
	if(ifGo)
	{
	  if(tc_->computeVelocityCommands(cmd_vel)){
	  last_valid_control_ = ros::Time::now();
	  vel_pub_.publish(cmd_vel);
	}
	}    
        else {
          ROS_ERROR("The local planner could not find a valid plan.");
          boost::unique_lock<boost::mutex> lock(planner_mutex_);
          planner_cond_.notify_one();
          lock.unlock();
          publishZeroVelocity();
         }

        break;
      default:
        ROS_ERROR("This case should never be reached, something is wrong, aborting");
        resetState();
        return true;
    }
    //we aren't done yet
    return false;
  }
  
  bool MoveBase::ifDirectlyToGoal(const geometry_msgs::PoseStamped start)
  {
   unsigned int mx,my;
   geometry_msgs::Point pp;//用于可视化
   float dist = distance(start,goal_);
   if(dist > 2) return false;
   float delta_x = (goal_.pose.position.x - start.pose.position.x)/dist*0.05;
   float delta_y = (goal_.pose.position.y - start.pose.position.y)/dist*0.05;
   geometry_msgs::PoseStamped checking_point_now = start;
   checking_set.points.clear();
   while(distance(checking_point_now,goal_) > 0.15)
   {
    checking_point_now.pose.position.x += delta_x;
    checking_point_now.pose.position.y += delta_y;
    planner_costmap_ros_->worldToMap(checking_point_now.pose.position.x, checking_point_now.pose.position.y, mx, my);
    
    pp.x = checking_point_now.pose.position.x;
    pp.y = checking_point_now.pose.position.y;
    checking_set.points.push_back(pp);//用于可视化
    if(planner_costmap_ros_->getCost(mx,my) > 50) 
    {
     marker_point_pub.publish(checking_set);
     return false;
    }
   }
   marker_point_pub.publish(checking_set); 
   return true;
  }
  
  void MoveBase::resetState(){
    // Reset statemachine
    state_ = PLANNING;
    publishZeroVelocity();
  }

}
