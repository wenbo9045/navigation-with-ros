/*
 *Brief discription:
 *A Better Monte Carlo Localization algorithm
 */
#include <algorithm>
#include <vector>
#include <map>
#include <cmath>
#include <iostream>
//#include "ceres/ceres.h"
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/PoseStamped.h>
// Signal handling
#include <signal.h>

#include "bmcl/map/map.h"
#include "bmcl/pf/pf.h"
#include "ros/assert.h"
//#include "bmcl/pf/map_cost_function.h"
// roscpp
#include "ros/ros.h"
// Messages that I need
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"
#include "nav_msgs/Odometry.h"
#include "std_srvs/Empty.h"

// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "message_filters/subscriber.h"
#include <assert.h>
using namespace std;
using namespace message_filters;

class BmclNode
{
 public:
 BmclNode();
 //~BmclNode();
 private:
 //ros parameter
 string global_frame_id_;
 string base_frame_id_;
 int max_sample_number_;//20181017
 int min_sample_number_;
 //boost
 boost::recursive_mutex configuration_mutex_;

 //ros basics
 ros::NodeHandle nh_;
 ros::NodeHandle private_nh_;//20181017 obtain the default parameters
 ros::Subscriber map_sub_;
 ros::Subscriber vel_sub_;
 //ros::Subscriber laser_scan_sub_;
 ros::Subscriber initial_pose_sub_;
 ros::Publisher pose_pub_;
 //tf message filter
 message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_; 
 tf::MessageFilter<sensor_msgs::LaserScan>* laser_scan_filter_;//used as the filter call back of the callback syncronizer
 //access to tf tree
 tf::TransformBroadcaster* tfb_;
 struct TransformListenerWrapper : public tf::TransformListener// Use a child class to get access to tf2::Buffer class inside of tf_
 {
   inline tf2_ros::Buffer &getBuffer() {return tf2_buffer_;}
 };
 TransformListenerWrapper* tf_; 
 //laser related
 void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
 //map related
 map_t* map_;//20181017 structure
 void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);
 void velReceived(const nav_msgs::Odometry& msg);
 void freeMapDependentMemory();
 void handleMapMessage(const nav_msgs::OccupancyGrid& msg);
 map_t* convertMap( const nav_msgs::OccupancyGrid& map_msg );
 static std::vector<std::pair<int,int> > free_space_indices;
 //initial pose and covariance
 double init_pose_[3];
 double init_cov_[3];
 bool locationInitialized;
 //particle filter
 pFilter *pf;//20181017 class
 //authenticate tf
 tf::Transform latest_tf_;
 bool latest_tf_valid_;
 //transform time tolerance
 ros::Duration transform_tolerance_;
 //pose according to odometry
 double poseInOdom[3];
 //current pose according to map
 double poseInMap[3];
 //if we should update and update condition
 bool ifUpdate;
 bool ifInit;
 double threshold_x;
 double threshold_y;
 double threshold_th;
 //handle initial pose
 void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
 //雷达出直线，直线栅格化
 bool map_processed;
 //判断是否是由于位移超过阈值导致测量更新
 bool dist_update;
 //机器人的实际速度与角速度
 float linear_vel_;
 float angular_vel_;
};

BmclNode::BmclNode():
		    private_nh_("~"),
		    map_processed(false),
		    ifUpdate(false),
		    ifInit(true),
		    dist_update(false),
		    latest_tf_valid_(false),
		    locationInitialized(false),
		    map_(NULL)
{
  private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));
  private_nh_.param("base_frame_id",base_frame_id_,std::string("base_link"));
  private_nh_.param("max_sample_number", max_sample_number_, 600);
  private_nh_.param("min_sample_number", min_sample_number_, 100);
  private_nh_.param("threshold_x",threshold_x,1.0);
  private_nh_.param("threshold_y",threshold_y,1.0);
  private_nh_.param("threshold_th",threshold_th,0.2);
  //subscriber,publisher,service,client
  vel_sub_ = nh_.subscribe("/odom",1,&BmclNode::velReceived,this);
  map_sub_ = nh_.subscribe("/map",1,&BmclNode::mapReceived,this);
  initial_pose_sub_ = nh_.subscribe("/initialpose",1,&BmclNode::initialPoseReceived,this);

  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("estimatedpose",1,true);
  //tf message filter
  tf_ = new TransformListenerWrapper();
  tfb_ = new tf::TransformBroadcaster();
  laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "scan", 100);
  laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_, *tf_, "odom", 100);
  laser_scan_filter_->registerCallback(boost::bind(&BmclNode::laserReceived,this, _1));
  transform_tolerance_.fromSec(0.1);
}

void BmclNode::laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan)
{ 
 if(!map_processed) return;
 if(map_ == NULL) 
 {
  ROS_ERROR("No map, open one!");
  return;
 }
 //Get laser pose according to base local frame
 tf::Stamped<tf::Pose> laser_pose;
 tf::Stamped<tf::Pose> poseInSelf(tf::Transform(tf::createIdentityQuaternion(),tf::Vector3(0,0,0)),ros::Time(),laser_scan->header.frame_id);
 try
 {
  tf_->transformPose(base_frame_id_,poseInSelf,laser_pose);
 }
 catch(tf::TransformException& e)
 {
  ROS_ERROR("Cannot transform from laser to base!");
  return;
 }
 //To figure out where the robot was when get the laser scan
 tf::Stamped<tf::Pose> robotPoseInOdom;
 tf::Stamped<tf::Pose> robotSelfPose(tf::Transform(tf::createIdentityQuaternion(),tf::Vector3(0,0,0)),laser_scan->header.stamp,base_frame_id_);
 try
 {
  tf_->transformPose("odom",robotSelfPose,robotPoseInOdom);
 }
 catch(tf::TransformException& e)
 {
  ROS_ERROR("Cannot retrieve the odometry when getting the laser scan");
  return;  
 }
 double yaw,pitch,roll;
  robotPoseInOdom.getBasis().getEulerYPR(yaw, pitch, roll);
 //Calculate the delta pose to check if we should update the measurement,
 //and to create the current sample set according to the base in map and delta pose.
 double delta_pose[3];
 delta_pose[0] = fabs(robotPoseInOdom.getOrigin().x()-poseInOdom[0]);
 delta_pose[1] = fabs(robotPoseInOdom.getOrigin().y()-poseInOdom[1]);
 delta_pose[2] = fabs(yaw-poseInOdom[2]);
 dist_update = false;
 if(delta_pose[0]>threshold_x||delta_pose[1]>threshold_y||delta_pose[2]>threshold_th)
 {
  ifUpdate = true;
  if(delta_pose[0]>threshold_x||delta_pose[1]>threshold_y)
  dist_update = true;
  poseInOdom[0] = robotPoseInOdom.getOrigin().x();
  poseInOdom[1] = robotPoseInOdom.getOrigin().y();
  poseInOdom[2] = yaw; 
 }

 //update measurement for each particle
if(ifUpdate||ifInit)
//if(1)
{
double secs =ros::Time::now().toSec();
if(locationInitialized)
{
  tf::Stamped<tf::Pose> robotPoseInMap;
  try
  {
   tf_->transformPose("map",robotSelfPose,robotPoseInMap);
   poseInMap[0] = robotPoseInMap.getOrigin().x();
   poseInMap[1] = robotPoseInMap.getOrigin().y();
   robotPoseInMap.getBasis().getEulerYPR(yaw, pitch, roll);
   poseInMap[2] = yaw;
  }
  catch(tf::TransformException& e)
  {
   ros::Time transform_expiration = (laser_scan->header.stamp + transform_tolerance_);
   tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),transform_expiration,global_frame_id_, "odom");
   tfb_->sendTransform(tmp_tf_stamped);
   ROS_WARN("Cannot retrieve base coordinate in map");
   return;  
  } 
}
float score = 0;
pf->SetInfo(laser_scan, Eigen::Vector3f(laser_pose.getOrigin().x(),laser_pose.getOrigin().y(),0));
vector<SparseScan> scans = pf->GenerateRotatedScans(Eigen::Vector3f(poseInMap[0],poseInMap[1],poseInMap[2]),20,0.5);
tuple<int,int,int> candidate = pf->scoreCandidates(scans,score);
pf->pf_mean.v[0] = poseInMap[0] + get<0>(candidate)*0.05;
pf->pf_mean.v[1] = poseInMap[1] + get<1>(candidate)*0.05;
pf->pf_mean.v[2] = poseInMap[2] + (get<2>(candidate)*0.5-20)*3.1415926/180;
double duration = ros::Time::now().toSec() - secs;
cout<<"time duration: "<<duration<<endl;
tf::Stamped<tf::Pose> odom_to_map;
  try
  {
   tf::Transform tmp_tf(tf::createQuaternionFromYaw(pf->pf_mean.v[2]),tf::Vector3(pf->pf_mean.v[0],pf->pf_mean.v[1],0));
   tf::Stamped<tf::Pose> tmp_tf_stamped(tmp_tf.inverse(),laser_scan->header.stamp,base_frame_id_);
   tf_->transformPose("odom",tmp_tf_stamped,odom_to_map);
  }
  catch(tf::TransformException)
  {
   ROS_ERROR("Failed to achieve base to odom transform");
   return;  
  }
  latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),tf::Point(odom_to_map.getOrigin()));
  ros::Time transform_expiration = (laser_scan->header.stamp + transform_tolerance_);
  tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),transform_expiration,global_frame_id_, "odom");//从map开始通过latest_tf_.inverse()转化到odom
  tfb_->sendTransform(tmp_tf_stamped);
  latest_tf_valid_ = true;
  ifUpdate = false;
  ifInit = false;
}

else if(latest_tf_valid_)
{
 ros::Time transform_expiration = (laser_scan->header.stamp + transform_tolerance_);
 tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),transform_expiration,global_frame_id_, "odom");
 tfb_->sendTransform(tmp_tf_stamped);
}
locationInitialized = true;
}
void BmclNode::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
 poseInMap[0] = msg->pose.pose.position.x;
 poseInMap[1] = msg->pose.pose.position.y;
 poseInMap[2] = tf::getYaw(msg->pose.pose.orientation);
 ifInit = true;
 locationInitialized = false;
}

void BmclNode::velReceived(const nav_msgs::Odometry& msg)
{
 linear_vel_ = msg.twist.twist.linear.x;
 angular_vel_ = msg.twist.twist.angular.z;
}

void BmclNode::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg)
{
  handleMapMessage(*msg);
}

void BmclNode::handleMapMessage(const nav_msgs::OccupancyGrid& msg)
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);

  ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
           msg.info.width,
           msg.info.height,
           msg.info.resolution);
  map_ = convertMap(msg);
  // Index of free space
  free_space_indices.resize(0);
  for(int i = 0; i < map_->size_x; i++)
    for(int j = 0; j < map_->size_y; j++)
      if(map_->cells[MAP_INDEX(map_,i,j)].occ_state == -1)
        free_space_indices.push_back(std::make_pair(i,j));
  map_update_cspace(map_, 1);
  line_update_cpace(map_,1);

  init_pose_[0] = 0.0;
  init_pose_[1] = 0.0;
  init_pose_[2] = 0.0;
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = init_pose_[0];
  pf_init_pose_mean.v[1] = init_pose_[1];
  pf_init_pose_mean.v[2] = init_pose_[2];
  
  poseInOdom[0] = init_pose_[0];
  poseInOdom[1] = init_pose_[1];
  poseInOdom[2] = init_pose_[2];
  poseInMap[0] = init_pose_[0];
  poseInMap[1] = init_pose_[1];
  poseInMap[2] = init_pose_[2];
  //created the particle filter
  pf = new pFilter(map_);
   map_processed = true;
}


std::vector<std::pair<int,int> > BmclNode::free_space_indices;

map_t* BmclNode::convertMap( const nav_msgs::OccupancyGrid& map_msg )
{
  map_t* map = map_alloc();
  ROS_ASSERT(map);
  
  map->size_x = map_msg.info.width;
  map->size_y = map_msg.info.height;
  map->scale = map_msg.info.resolution;
  map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
  // Convert to player format
  map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
  ROS_ASSERT(map->cells);
  for(int i=0;i<map->size_x * map->size_y;i++)
  {
    if(map_msg.data[i] == 0)
      map->cells[i].occ_state = -1;
    else if(map_msg.data[i] == 100)
      map->cells[i].occ_state = +1;
    else
      map->cells[i].occ_state = 0;
  }
  return map;
}

void BmclNode::freeMapDependentMemory()
{
  if( map_ != NULL ) {
    map_free(map_);
    map_ = NULL;
  }
}

int main(int argc, char**argv)
{
 std::cout<<"Bmcl start"<<std::endl;
 ros::init(argc,argv,"bmcl");
 BmclNode B;
 ros::spin();
}
