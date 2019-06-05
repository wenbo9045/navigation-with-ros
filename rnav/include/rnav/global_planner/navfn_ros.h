#ifndef GLOBAL_NAVFN_ROS_H_
#define GLOBAL_NAVFN_ROS_H_

#include <ros/ros.h>
#include "navfn.h"
#include "rnav/nav_map/nav_map.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <nav_msgs/GetPlan.h>
#include <navfn/potarr_point.h>
#include <pcl_ros/publisher.h>

namespace navfn {

  class NavfnROS{
    public:
      /**
       * @brief  Default constructor for the NavFnROS object
       */
      NavfnROS();

      NavfnROS(std::string name, navMap::navMap* costmap_ros);

      NavfnROS(std::string name, navMap::navMap* costmap, std::string global_frame);

      void initialize(std::string name, navMap::navMap* costmap_ros);

      void initialize(std::string name, navMap::navMap* costmap, std::string global_frame);

      bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan);
          
      bool computePotential(const geometry_msgs::Point& world_point);

      bool getPlanFromPotential(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      double getPointPotential(const geometry_msgs::Point& world_point);


      bool validPointPotential(const geometry_msgs::Point& world_point);

      bool validPointPotential(const geometry_msgs::Point& world_point, double tolerance);

      void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a);

      ~NavfnROS(){}

      bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

    protected:

      navMap::navMap* costmap_;
      boost::shared_ptr<NavFn> planner_;
      ros::Publisher plan_pub_;
      pcl_ros::Publisher<PotarrPoint> potarr_pub_;
      bool initialized_, allow_unknown_, visualize_potential_;


    private:
      inline double sq_distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2){
        double dx = p1.pose.position.x - p2.pose.position.x;
        double dy = p1.pose.position.y - p2.pose.position.y;
        return dx*dx +dy*dy;
      }

      void mapToWorld(double mx, double my, double& wx, double& wy);
      void clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my);
      double planner_window_x_, planner_window_y_, default_tolerance_;
      std::string tf_prefix_;
      boost::mutex mutex_;
      ros::ServiceServer make_plan_srv_;
      std::string global_frame_;
  };
};

#endif
