#include "rnav/local_planner/goal_functions.h"

namespace tri_local_planner {



  double getGoalOrientationAngleDifference(const tf::Stamped<tf::Pose>& global_pose, double goal_th) {
    double yaw = tf::getYaw(global_pose.getRotation());
    return angles::shortest_angular_distance(yaw, goal_th);
  }

  void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub) {
    //given an empty path we won't do anything
    if(path.empty())
      return;

    //create a path message
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());
    gui_path.header.frame_id = path[0].header.frame_id;
    gui_path.header.stamp = path[0].header.stamp;

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++){
      gui_path.poses[i] = path[i];
    }

    pub.publish(gui_path);
  }

  void prunePlan(const tf::Stamped<tf::Pose>& global_pose, std::vector<geometry_msgs::PoseStamped>& plan, std::vector<geometry_msgs::PoseStamped>& global_plan){
    //ROS_INFO("global:%i,transform:%i",global_plan.size(),plan.size());
    ROS_ASSERT(global_plan.size() >= plan.size());
    if(global_plan.empty()||plan.empty()||global_plan.size() < plan.size()){
        //ROS_WARN("transformed_plan.size > global_plan.size");
        return;
    }
    std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();
    std::vector<geometry_msgs::PoseStamped>::iterator global_it = global_plan.begin();
    while(it != plan.end()){
      const geometry_msgs::PoseStamped& w = *it;
      // Fixed error bound of 2 meters for now. Can reduce to a portion of the map size or based on the resolution
      double x_diff = global_pose.getOrigin().x() - w.pose.position.x;
      double y_diff = global_pose.getOrigin().y() - w.pose.position.y;
      double distance_sq = x_diff * x_diff + y_diff * y_diff;
      if(distance_sq < 1){
        ROS_DEBUG("Nearest waypoint to <%f, %f> is <%f, %f>\n", global_pose.getOrigin().x(), global_pose.getOrigin().y(), w.pose.position.x, w.pose.position.y);
        break;
      }
      it = plan.erase(it);
      global_it = global_plan.erase(global_it);
    }
  }
  

void transformGoalPose(
      const tf::TransformListener& tf,
      const geometry_msgs::PoseStamped& plan_goal_pose,
      tf::Stamped<tf::Pose>& goal_pose,
      const std::string& global_frame){
   // ROS_INFO("plan_fram:%s,global_frame:%s",plan_goal_pose.header.frame_id.c_str(),global_frame.c_str());   
    try{
      tf::StampedTransform transform;
      tf.waitForTransform(global_frame, ros::Time::now(),
                          plan_goal_pose.header.frame_id, plan_goal_pose.header.stamp,
                          plan_goal_pose.header.frame_id, ros::Duration(0.5));
      tf.lookupTransform(global_frame, ros::Time(),
                         plan_goal_pose.header.frame_id, plan_goal_pose.header.stamp,
                         plan_goal_pose.header.frame_id, transform);

      poseStampedMsgToTF(plan_goal_pose, goal_pose);
      goal_pose.setData(transform * goal_pose);
      goal_pose.stamp_ = transform.stamp_;
      goal_pose.frame_id_ = global_frame;
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return;
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return;
    }           
}
  void transformPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan,std::vector<geometry_msgs::PoseStamped>& transformed_plan)
  {
   transformed_plan.clear();
   geometry_msgs::PoseStamped first_position(global_plan[0]);
   float distance_threshold = 2;
   float dist = 0;
   for(int num = 0; num < global_plan.size(); ++num)
   {
    dist = distance(first_position,global_plan[num]);
    //ROS_WARN("DISTANCE %f",dist);
    if(dist >distance_threshold) {break;}
    transformed_plan.push_back(global_plan[num]);
   }
   //transformed_plan = global_plan; 
  }

};
