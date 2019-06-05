/*
 *Brief discription:
 *A Better mapping algorithm
 */
#include <algorithm>
#include <vector>
#include <map>
#include <cmath>
#include <iostream>
#include <tuple>
//#include "ceres/ceres.h"
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/PoseStamped.h>
// Signal handling
#include <signal.h>
#include "ros/ros.h"
#include "ros/assert.h"
// Messages that I need
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_srvs/Empty.h"
// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/tf.h"
#include <visualization_msgs/Marker.h>
#include "ymapping/transform/rigid_transform.h"
#include "ymapping/map/xy_index.h"
#include "ymapping/map/map_limits.h"
#include "ymapping/map/probability_grid.h"
#include "ymapping/map/submaps.h"
#include "ymapping/ploc/ploc.h"
#include <iostream>
#include <mutex>
class Ymapping
{
 public:
 Ymapping();
 ~Ymapping();
 private:
 float linear_vel_ = 0;
 float angular_vel_ = 0;
 int loc_time = 0;
 //access to tf tree
 tf::TransformBroadcaster* tfb_;
 struct TransformListenerWrapper : public tf::TransformListener// Use a child class to get access to tf2::Buffer class inside of tf_
 {
   inline tf2_ros::Buffer &getBuffer() {return tf2_buffer_;}
 };
 TransformListenerWrapper* tf_; 
 ros::NodeHandle n_;
 ros::Publisher raw_marker_pub;//原始点云
 ros::Publisher transformed_marker_pub;//变换过的点云
 ros::Publisher map_pub;
 ros::Subscriber laser_scan_sub_;
 ros::Subscriber odom_sub_;
 ros::Subscriber initial_pose_sub_;
 void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
 void odomReceived(const nav_msgs::Odometry& odom);
 std::mutex vel;
 nav_msgs::OccupancyGrid map;
 /*For testing and Visualization*/
 visualization_msgs::Marker raw_point_cloud;
 visualization_msgs::Marker transformed_point_cloud;
 mapping::Submap sMap;
 //只用来测试
 void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
 {
  Eigen::Array2i index = sMap.probability_grid().limits().GetCellIndex(Eigen::Vector2f(msg->pose.pose.position.x,msg->pose.pose.position.y));
  sMap.probability_grid().getLocValue(index);
 }
 void GenerateMap(const nav_msgs::OccupancyGrid& map)
 {
  ROS_INFO("Received a %d X %d map @ %.3f m/pix",
               map.info.width,
               map.info.height,
               map.info.resolution);


  std::string mapdatafile = "/home/yuanrupeng/Ymapping.pgm";
  ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
  FILE* out = fopen(mapdatafile.c_str(), "w");
  if (!out)
  {
   ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
   return;
  }

  fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
  map.info.resolution, map.info.width, map.info.height);
  for(unsigned int y = 0; y < map.info.height; y++) {
    for(unsigned int x = 0; x < map.info.width; x++) {
     unsigned int i = x + (map.info.height - y - 1) * map.info.width;
     if (map.data[i] == 0) { //occ [0,0.1)
     fputc(254, out);
     } else if (map.data[i] == +100) { //occ (0.65,1]
       fputc(000, out);
     } else { //occ [0.1,0.65]
      fputc(205, out);
    }
   }
  }

  fclose(out);
  std::string mapmetadatafile = "/home/yuanrupeng/Ymapping.yaml";
  ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
  FILE* yaml = fopen(mapmetadatafile.c_str(), "w");
  double yaw = 0;
      
  fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
  mapdatafile.c_str(), map.info.resolution, map.info.origin.position.x, map.info.origin.position.y, yaw);

  fclose(yaml);
  ROS_INFO("Done\n");
 }
};

 Ymapping::~Ymapping()
 {
  for(int i = 0; i<map.data.size(); ++i)
  {
   if(map.data[i] < 10) map.data[i]=-1;
   else if(map.data[i] < 50) map.data[i]=0;
   else map.data[i] = 100;
  }
  GenerateMap(map);
  std::cout<<"stop"<<std::endl;
  delete tf_;
  delete tfb_;
 }
Ymapping::Ymapping():sMap(mapping::MapLimits(0.05,Eigen::Vector2d(-1,-1),mapping::CellLimits(10,10)),transform::Rigid2f(transform::Rigid2f::Vector(-1,-1),0),0.7,0.2)
{
  map.header.frame_id = "/map";
  map.info.resolution = 0.05;
  //tf message filter
  tf_ = new TransformListenerWrapper();
  tfb_ = new tf::TransformBroadcaster();
  raw_marker_pub = n_.advertise<visualization_msgs::Marker>("raw_laser",10);
  transformed_marker_pub = n_.advertise<visualization_msgs::Marker>("transform_laser",10);
  map_pub = n_.advertise<nav_msgs::OccupancyGrid>("map",10);
  laser_scan_sub_ = n_.subscribe("scan",1,&Ymapping::laserReceived,this);  
  odom_sub_ = n_.subscribe("odom",1,&Ymapping::odomReceived,this);
  initial_pose_sub_ = n_.subscribe("/initialpose",1,&Ymapping::initialPoseReceived,this);
  raw_point_cloud.header.frame_id = "/base_link";
  raw_point_cloud.type = visualization_msgs::Marker::POINTS;
  raw_point_cloud.scale.x = 0.05;
  raw_point_cloud.scale.y = 0.05;
  raw_point_cloud.color.b = 1.0;
  raw_point_cloud.color.a = 1.0;
  
  transformed_point_cloud.header.frame_id = "/map";
  transformed_point_cloud.type = visualization_msgs::Marker::POINTS;
  transformed_point_cloud.scale.x = 0.05;
  transformed_point_cloud.scale.y = 0.05;
  transformed_point_cloud.color.g = 1.0;
  transformed_point_cloud.color.a = 1.0;
}
void Ymapping::odomReceived(const nav_msgs::Odometry& odom)
{
 linear_vel_ = odom.twist.twist.linear.x;
 angular_vel_ = odom.twist.twist.angular.z;
}

/*
 * 1. 对scan进行处理得到以base_link为坐标的pointcloud
 * 2. 通过已有地图，进行定位。如果没有则直接建图,发布定位的broadcaster
 * 3. 判断是否需要扩大地图，还是直接在原来地图上绘制
 * 4. 使用bressenham绘制直线，利用hit table和miss table来更新数值
 */
void Ymapping::laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
 tf::Stamped<tf::Pose> odom_to_map;
 ros::Duration transform_tolerance_;
 transform_tolerance_.fromSec(0.05);
 ros::Time transform_expiration = (laser_scan->header.stamp + transform_tolerance_);
 /*1. 对scan进行处理得到以base_link为坐标的pointcloud*/
 //baselink变换到雷达的变换
 tf::Stamped<tf::Pose> laser_pose;
 tf::Stamped<tf::Pose> poseInSelf(tf::Transform(tf::createIdentityQuaternion(),tf::Vector3(0,0,0)),ros::Time(),laser_scan->header.frame_id);
 try
 {
  tf_->transformPose("base_link",poseInSelf,laser_pose);
 }
 catch(tf::TransformException& e)
 {
  ROS_ERROR("Cannot transform from laser to base!");
  return;
 }
 
 if(sMap.SubmapStatus())
 {
  //得到base_link相对于map的变换
  tf::Stamped<tf::Pose> initialbaseOrigin;
  tf::Stamped<tf::Pose> initialbaseInSelf(tf::Transform(tf::createIdentityQuaternion(),tf::Vector3(0,0,0)),ros::Time(),"base_link"); 
  try
  {
    tf_->transformPose("map",initialbaseInSelf,initialbaseOrigin);
  }
  catch(tf::TransformException& e)
  {
    ROS_ERROR("Cannot transform from map to base!");
    return;
  }

  Ploc loc(laser_scan,Eigen::Vector3f(laser_pose.getOrigin().x(), laser_pose.getOrigin().y(),0),&sMap.probability_grid());
  double secs =ros::Time::now().toSec();
  std::vector<SparseScan> ptest = loc.GenerateRotatedScans(Eigen::Vector3f(initialbaseOrigin.getOrigin().x(),
                                                                              initialbaseOrigin.getOrigin().y(),
                                                                              tf::getYaw(initialbaseOrigin.getRotation())), 20, 0.5);
  float score = 0;
  tuple<int,int,int> candidate = loc.scoreCandidates(ptest,score);
  std::cout<<"current score: "<<score<<std::endl;
  double duration = ros::Time::now().toSec() - secs;
  std::cout<<"time duration: "<<duration<<std::endl;
  double loc_x = initialbaseOrigin.getOrigin().x() + get<0>(candidate)*0.05;
  double loc_y = initialbaseOrigin.getOrigin().y() + get<1>(candidate)*0.05;
  double loc_angle = tf::getYaw(initialbaseOrigin.getRotation()) + (get<2>(candidate)*0.5-20)*3.1415926/180;
  try
  {
   tf::Transform tmp_tf(tf::createQuaternionFromYaw(loc_angle),tf::Vector3(loc_x,loc_y,0));
   tf::Stamped<tf::Pose> tmp_tf_stamped(tmp_tf.inverse(),laser_scan->header.stamp,"base_link");
   tf_->transformPose("odom",tmp_tf_stamped,odom_to_map);
  }
  catch(tf::TransformException)
  {
   return;  
  }
  tf::Transform latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),tf::Point(odom_to_map.getOrigin()));
  //从map开始通过latest_tf_.inverse()转化到odom
  tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),transform_expiration,"map", "odom");
  tfb_->sendTransform(tmp_tf_stamped);
 }
  /*TODO: 2. 通过已有地图，进行定位。如果没有则直接建图*/
  /*发布定位的TF变换*/
  else
  {
   tf::Transform latest_tf_(tf::createQuaternionFromYaw(0),tf::Vector3(0,0,0));
   //从map开始通过latest_tf_.inverse()转化到odom
   tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),transform_expiration,"map", "odom");
   tfb_->sendTransform(tmp_tf_stamped);
   /*用于测试，不定位，认为map和odom重合*/
  }
  
  if(angular_vel_ == 0)
  {
   loc_time++;//std::cout<<"time: "<<loc_time<<std::endl;
   if(loc_time == 4)
   {
    loc_time = 0;
    //得到laser到base_link的坐标变换
    transform::Rigid2f Transform(transform::Rigid2f::Vector(laser_pose.getOrigin().x(), laser_pose.getOrigin().y()),0);
    //得到在base_link下的雷达点云，经过稀疏化处理
    sensor::PointCloud pp = sensor::scanToPointCloud(Transform,laser_scan);
    //得到base_link相对于map的变换
    tf::Stamped<tf::Pose> baseOrigin;
    tf::Stamped<tf::Pose> baseInSelf(tf::Transform(tf::createIdentityQuaternion(),tf::Vector3(0,0,0)),ros::Time(),"base_link"); 
    try
    {
     tf_->transformPose("map",baseInSelf,baseOrigin);
    }
    catch(tf::TransformException& e)
    {
     ROS_ERROR("Cannot transform from map to base!");
     return;
    }
    //得到eigen变换的map到base_link的变换
    transform::Rigid2f trans(transform::Rigid2f::Vector(baseOrigin.getOrigin().x(),baseOrigin.getOrigin().y()),tf::getYaw(baseOrigin.getRotation()));
    pp = sensor::transformPointCloud(trans,pp);
    //得到laser相对于map的变换
    tf::Stamped<tf::Pose> laserOrigin;
    tf::Stamped<tf::Pose> laserInSelf(tf::Transform(tf::createIdentityQuaternion(),tf::Vector3(0,0,0)),ros::Time(),"laser"); 
    try
    {
     tf_->transformPose("map",laserInSelf,laserOrigin);
    }
    catch(tf::TransformException& e)
    {
      ROS_ERROR("Cannot transform from map to laser!");
     return;
    }
    Eigen::Vector2f laser_in_map(laserOrigin.getOrigin().x(),laserOrigin.getOrigin().y());
    //更新地图，new_limits，new_cells，new_map_cells，new_cell_value
    sMap.MapCheck(pp);
    sMap.InsertRangeData(pp,laser_in_map);
    map.info.width = sMap.probability_grid().limits().cell_limits().num_x_cells;
    map.info.height = sMap.probability_grid().limits().cell_limits().num_y_cells;
    map.info.origin.position.x = sMap.probability_grid().limits().min().x();
    map.info.origin.position.y = sMap.probability_grid().limits().min().y();
    map.data = sMap.probability_grid().map_cell();
    map_pub.publish(map);
    sMap.Finish();
   }
  }
}

int main(int argc, char** argv)
{
 ros::init(argc,argv,"ymapping");
 ROS_INFO("Start Mapping");
 Ymapping Y;
 ros::spin();
 return 0;
}
