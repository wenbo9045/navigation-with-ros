#ifndef NAV_MAP_H_
#define NAV_MAP_H_
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
namespace navMap
{
extern vector<vector<float> >* cached_distance;
extern vector<vector<int> >* cached_cost;
extern vector<int> *cost_translation_table;
class navMap
{
 public:
 navMap()
 {
  ROS_ERROR("construct navmap");
  tf_ = new TransformListenerWrapper();
  tfb_ = new tf::TransformBroadcaster();
  nav_map_pub_ = n_.advertise<nav_msgs::OccupancyGrid>("nav_map",1);
  map_sub_ = n_.subscribe("/map",1,&navMap::mapReceived,this);
  laser_sub_ = n_.subscribe("/scan",1,&navMap::laserReceived,this);
  loc_test_sub_ = n_.subscribe("/move_base_simple/goal",1,&navMap::locReceived,this);
  pub_thread_ = new boost::thread(boost::bind(&navMap::pubThread, this));
  computeCaches();
 };
 ~navMap()
 {
  ROS_ERROR("delete navmap");
  delete tf_;
  delete tfb_;
  pub_thread_->interrupt();
  pub_thread_->join();
  delete pub_thread_;
 }
 /*********************************接口函数*****************************/
 unsigned char getCost(int mx,int my)
 {
  pair<int,int> index(mx,my);
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

unsigned char* getCharMap() const
{
  unsigned char* costmap =  new unsigned char[num_x_*num_y_];
  for(int i = 0; i < value_map_.size(); ++i)
  costmap[i] = value_map_[i];
  return costmap;
}

void setCost(unsigned int mx, unsigned int my, unsigned char cost)
{
 value_map_[ToFlatIndex(mx, my)] = cost;
}

double getOriginX() const
{
  return origin_.x();
}

double getOriginY() const
{
  return origin_.y();
}
  /**************************************************************/
 
 
 void pubThread()
 {
   ros::Rate r(20);
  while(n_.ok())
  {
   if(!if_pub) break;
   if(map_processed)
   nav_map_pub_.publish(nav_map_);
      r.sleep();
  }
 }

 
 void locReceived(const geometry_msgs::PoseStampedPtr& msg)
 {
  pair<int,int> index = FetchCellIndex(msg->pose.position.x,msg->pose.position.y);
  int value = getCost(index.first,index.second);
  ROS_ERROR("value: %d",(unsigned int)value);
 }
  
 void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg)
 {
  num_x_ = msg->info.width;
  num_y_ = msg->info.height;
  resolution_ = msg->info.resolution;

  ROS_ERROR("resolution %f, num_x %d, num_y %d",resolution_,num_x_,num_y_);
  origin_ << msg->info.origin.position.x,msg->info.origin.position.y;
  nav_map_.header = msg->header;
  nav_map_.info = msg->info;
  nav_map_.data.reserve(msg->data.size());
  value_map_.reserve(msg->data.size());

  //TODO: value_map的膨胀。
  std::vector<bool> seen_tmp(num_x_*num_y_,false);
  seen_ = seen_tmp;
  assert(msg->data.size() == seen_.size());
  for(int i = 0; i < msg->data.size(); ++i)
  {
   if(msg->data[i]==100)
   {
    nav_map_.data.push_back(100);
    value_map_.push_back(254);
    inflation_queue[0.0].push_back(CellData(i,i%num_x_,i/num_x_,i%num_x_,i/num_x_));
    assert(i == ToFlatIndex(i%num_x_,i/num_x_));
    seen_[i] = true;
   }
   else
   {
    nav_map_.data.push_back(0);
    value_map_.push_back(0);
   }
  }
  std::map<double,vector<CellData> >::iterator queue_iter;
  double secs =ros::Time::now().toSec();
  for(queue_iter=inflation_queue.begin();queue_iter!=inflation_queue.end();queue_iter++)
  {
    for(int i=0;i<(*queue_iter).second.size();i++)
    {
     const CellData& current_cell = ((*queue_iter).second)[i];
     unsigned int index = current_cell.index_;
     unsigned int mx = current_cell.x_;
     unsigned int my = current_cell.y_;
     unsigned int sx = current_cell.src_x_;
     unsigned int sy = current_cell.src_y_;
     if (mx > 0)
       enqueue(index - 1, mx - 1, my, sx, sy);
     if (my > 0)
       enqueue(index - num_x_, mx, my - 1, sx, sy);
     if (mx < num_x_ - 1)
       enqueue(index + 1, mx + 1, my, sx, sy);
     if (my < num_y_ - 1)
       enqueue(index + num_x_, mx, my + 1, sx, sy);
    }
  }
  double duration = ros::Time::now().toSec() - secs;
  //std::cout<<"time duration: "<<duration<<std::endl;
  nav_map_pub_.publish(nav_map_);
  map_processed = true;
 }

 void laserReceived(const sensor_msgs::LaserScanPtr& laser_scan)
 {
   if_pub = false;
  double secs =ros::Time::now().toSec();
  int index;
  if(!map_processed) return;
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
//清除上次更新
  finishUpdate(indicesValue);
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
   assert(index < nav_map_.data.size());
   if(get<2>(scanIndexInMap[i]) > value_map_[index])
   {
    indicesValue.emplace_back(index,value_map_[index]);
    value_map_[index] = get<2>(scanIndexInMap[i]);
    nav_map_.data[index] = (*cost_translation_table)[value_map_[index]];
   }
  }
  nav_map_pub_.publish(nav_map_);

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
    nav_map_.data[indicesValue[i].first] = (*cost_translation_table)[cost];
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
     nav_map_.data[index] = (*cost_translation_table)[cost];
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
 float resolution_;
 int num_x_, num_y_;
 Eigen::Array2f origin_;
 nav_msgs::OccupancyGrid nav_map_;
 //存储每一个cell的value值，这些值不可以高过255.
 std::vector<int> value_map_;
 ros::NodeHandle n_;
 ros::Subscriber map_sub_;
 ros::Subscriber laser_sub_;
 ros::Publisher nav_map_pub_;
 ros::Subscriber loc_test_sub_;
 bool map_processed = false;
 std::map<double,vector<CellData> > inflation_queue;
 std::vector<bool> seen_;
 float cell_inflation_radius_ = 10;//以栅格为单位的膨胀半径，10等于50cm
 vector<pair<int,int> > indicesValue;
 boost::thread* pub_thread_;
 bool if_pub = true;
};
}

#endif 
