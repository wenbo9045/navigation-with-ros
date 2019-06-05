#ifndef LOCAL_MAP_H_
#define LOCAL_MAP_H_
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "rnav/transform/rigid_transform.h"
#include "point_cloud.h"
#include <vector>
#include <queue>
#include <tuple>
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/tf.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <utility>
#include <visualization_msgs/Marker.h>
#include <assert.h>
#include <map>
#include "cached_info.h"
#include "geometry_msgs/PoseStamped.h"
typedef std::vector<Eigen::Vector2f> PointCloud;
namespace localMap
{

extern vector<vector<float> >* cached_distance;
extern vector<vector<int> >* cached_cost;
extern vector<int>* cost_translation_table;
class localMap
{
 public:
 localMap()
 {
  ROS_ERROR("construct localmap");
  tf_ = new TransformListenerWrapper();
  tfb_ = new tf::TransformBroadcaster();
  local_map_pub_ = n_.advertise<nav_msgs::OccupancyGrid>("local_map",1);
  laser_sub_ = n_.subscribe("/scan",1,&localMap::laserReceived,this);
  loc_test_sub_ = n_.subscribe("/move_base_simple/goal",1,&localMap::locReceived,this);
  pub_thread_ = new boost::thread(boost::bind(&localMap::pubThread, this));
  value_map_=vector<int>(num_x_*num_y_,0);
  local_map_.data = vector<signed char>(num_x_*num_y_,0);
  local_map_.info.width = num_x_;
  local_map_.info.height = num_y_;
  local_map_.info.resolution = resolution_;
  computeCaches();
 };
 ~localMap()
 {
  ROS_ERROR("delete localmap");
  delete tf_;
  delete tfb_;
  pub_thread_ -> interrupt();
  pub_thread_ -> join();
  delete pub_thread_;
 }
 /*********************************接口函数*****************************/
 unsigned char getCost(int mx,int my)
 {
  pair<int,int> index(mx,my);
  //ROS_ERROR("x_index %d, y_index %d, num_x %d, num_y %d",index.first,index.second,num_x_,num_y_);
  if(index.first > num_x_ || index.second > num_y_ || index.first < 0 || index.second < 0) return NO_INFORMATION;
  else return value_map_[ToFlatIndex(index)];
 }
 
 int getSizeInCellsX() {return num_x_;}
 int getSizeInCellsY() {return num_y_;}
 float getResolution() {return resolution_;}
 bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const
{
  if (wx < origin_.x() || wy < origin_.y())
    return false;

  mx = (int)((wx - origin_.x()) / resolution_);
  my = (int)((wy - origin_.y()) / resolution_);

  if (mx < num_x_ && my < num_y_)
    return true;

  return false;
}
void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const
{
  wx = origin_.x() + (mx + 0.5) * resolution_;
  wy = origin_.y() + (my + 0.5) * resolution_;
}

bool getRobotPose(tf::Stamped<tf::Pose>& global_pose) const
{
  global_pose.setIdentity();
  tf::Stamped < tf::Pose > robot_pose;
  robot_pose.setIdentity();
  robot_pose.frame_id_ = "base_link";
  robot_pose.stamp_ = ros::Time();
  ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

  // get the global pose of the robot
  try
  {
    tf_->transformPose("map", robot_pose, global_pose);
  }
  catch (tf::LookupException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf::ConnectivityException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf::ExtrapolationException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
    return false;
  }

  return true;
}

  /**************************************************************/
 void pubThread()
 {
  ros::Rate r(20);
  while(n_.ok())
  {
   if(!if_pub) break;
    //得到base_link相对于map的变换
 tf::Stamped<tf::Pose> initialbaseOrigin;
 tf::Stamped<tf::Pose> initialbaseInSelf(tf::Transform(tf::createIdentityQuaternion(),tf::Vector3(0,0,0)),ros::Time(),"base_link"); 
  try
  {
   tf_->transformPose("map",initialbaseInSelf,initialbaseOrigin);
  }
  catch(tf::TransformException& e)
  {
      r.sleep();
   continue;
  }
  origin_ = updateOrigin(initialbaseOrigin);
  //更新有关COSTMAP的所有信息
  local_map_.info.origin.position.x = origin_.x();
  local_map_.info.origin.position.y = origin_.y();
  local_map_.header.seq = 1;
  local_map_.header.stamp = ros::Time(0);
  local_map_.header.frame_id = "map";
   local_map_pub_.publish(local_map_);
   r.sleep();
  }
 }

 void locReceived(const geometry_msgs::PoseStampedPtr& msg)
 {
  pair<int,int> index = FetchCellIndex(msg->pose.position.x,msg->pose.position.y);
  int value = getCost(index.first,index.second);
  ROS_ERROR("value: %d",(unsigned int)value);
 }
  
 
 Eigen::Array2f updateOrigin(tf::Stamped<tf::Pose> position)
 {
  float base_origin_x = position.getOrigin().x();
  float base_origin_y = position.getOrigin().y();
  Eigen::Array2f origin(base_origin_x - num_x_*resolution_/2, base_origin_y - num_y_*resolution_/2);
  return origin;
 }
 
 void laserReceived(const sensor_msgs::LaserScanPtr& laser_scan)
 {
  if_pub = false;
  double secs =ros::Time::now().toSec();
  int index;
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
  update_counter_++;
  if(update_counter_ < 4) return;
  update_counter_ =0;
  origin_ = updateOrigin(initialbaseOrigin);
  //更新有关COSTMAP的所有信息
  local_map_.info.origin.position.x = origin_.x();
  local_map_.info.origin.position.y = origin_.y();
  local_map_.header.seq = laser_scan->header.seq;
  local_map_.header.stamp = laser_scan->header.stamp;
  local_map_.header.frame_id = "map";
  finishUpdate(indicesValue);//清除上次更新
  indicesValue.clear();
  pointCloud pCloud(laser_scan,Eigen::Vector3f(laser_pose.getOrigin().x(),laser_pose.getOrigin().y(),0),origin_,num_x_,num_y_,resolution_);
  vector<tuple<int,int,int> > scanIndexInMap = pCloud.indexAffected(Eigen::Vector3f(initialbaseOrigin.getOrigin().x(),
                                                                 initialbaseOrigin.getOrigin().y(),
                                                                 tf::getYaw(initialbaseOrigin.getRotation())));

  vector<int> test;
  for(int i = 0; i < scanIndexInMap.size(); ++i)
  {
   index = ToFlatIndex(get<0>(scanIndexInMap[i]),get<1>(scanIndexInMap[i]));
   test.push_back(index);
   if(index >= local_map_.data.size()) ROS_ERROR("index: %d, x_index: %d, y_index: %d map size: %d",index,get<0>(scanIndexInMap[i]),get<1>(scanIndexInMap[i]),local_map_.data.size());
   assert(index < local_map_.data.size());
   if(get<2>(scanIndexInMap[i]) > value_map_[index])
   {
    indicesValue.emplace_back(index,value_map_[index]);
    value_map_[index] = get<2>(scanIndexInMap[i]);
    local_map_.data[index] = (*cost_translation_table)[value_map_[index]];
   }
  }
  local_map_pub_.publish(local_map_);

  double duration = ros::Time::now().toSec() - secs;
  //std::cout<<"time duration: "<<duration<<std::endl;
 }
 
 void finishUpdate(vector<pair<int,int> > indicesValue)
 {
  int cost;
  for(int i = 0; i < indicesValue.size(); ++i)
  {
    cost = indicesValue[i].second;
    value_map_[indicesValue[i].first] = cost;
    local_map_.data[indicesValue[i].first] = (*cost_translation_table)[cost];
  }
 }
  
 
 private:
 //得到一个点相对于地图原点的x,y索引值
  std::pair<int,int> FetchCellIndex(const float& pt_x, const float& pt_y) const {
    return std::make_pair(
       std::floor((pt_x - origin_.x()) / resolution_ + 0.5) ,
       std::floor((pt_y - origin_.y()) / resolution_ + 0.5)        
        );
  }
 //根据x,y索引值计算得到index索引值
 inline int ToFlatIndex(const pair<int,int> cell_index) const {
    return num_x_ * cell_index.second + cell_index.first;
  }

 inline int ToFlatIndex(int cell_index_x, int cell_index_y) const {
    return num_x_ * cell_index_y + cell_index_x;
  }
  
  inline double distanceLookup(int mx, int my, int src_x, int src_y)
  {
    unsigned int dx = abs(mx - src_x);
    unsigned int dy = abs(my - src_y);
    //ROS_ERROR("dist %f",cached_distance[dx][dy]);
    return (*cached_distance)[dx][dy];
  }
  
  inline int costLookup(int mx, int my, int src_x, int src_y)
  {
    unsigned int dx = abs(mx - src_x);
    unsigned int dy = abs(my - src_y);
    return (*cached_cost)[dx][dy];
  }
  
  inline void enqueue(unsigned int index, unsigned int mx, unsigned int my,
                                      unsigned int src_x, unsigned int src_y)
  {
    if (!seen_[index])
    {
     double distance = distanceLookup(mx, my, src_x, src_y);
     if (distance > cell_inflation_radius_) return;
     inflation_queue[distance].emplace_back(index, mx, my, src_x, src_y);
     int cost = costLookup(mx, my, src_x, src_y);
     local_map_.data[index] = (*cost_translation_table)[cost];
     value_map_[index] = cost;
     assert(value_map_[index]<=254);
     seen_[index] = true;
    }
  }
  
  
  //access to tf tree
 tf::TransformBroadcaster* tfb_;
 struct TransformListenerWrapper : public tf::TransformListener// Use a child class to get access to tf2::Buffer class inside of tf_
 {
   inline tf2_ros::Buffer &getBuffer() {return tf2_buffer_;}
 };
 TransformListenerWrapper* tf_; 
 float resolution_ = 0.05;
 int num_x_ = 100;
 int num_y_ = 100;
 Eigen::Array2f origin_;
 nav_msgs::OccupancyGrid local_map_;
 std::vector<int> value_map_;//存储每一个cell的value值，这些值不可以高过255.
 ros::NodeHandle n_;
 ros::Subscriber laser_sub_;
 ros::Publisher local_map_pub_;
 ros::Subscriber loc_test_sub_;
 std::map<double,vector<CellData> > inflation_queue;
 std::vector<bool> seen_;
 float cell_inflation_radius_ = 10;//以栅格为单位的膨胀半径，10等于50cm
 vector<pair<int,int> > indicesValue;
 boost::thread* pub_thread_;
 bool if_pub = true;
 int update_counter_ = 0;
};
}

#endif 
