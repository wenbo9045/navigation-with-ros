#ifndef LOCAL_POINT_CLOUD_H_
#define LOCAL_POINT_CLOUD_H_
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <map>
#include "tf/tf.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <tuple>
#include <iostream>
#include "cached_info.h"
using namespace std;
typedef std::vector<pair<int,int> > SparseScan;
namespace localMap
{
class pointCloud
{
 public:
 pointCloud(const sensor_msgs::LaserScanPtr& l,Eigen::Vector3f l_pose,Eigen::Array2f origin,int num_x, int num_y, float resolution):
 laser_scan(l),laser_pose_(l_pose),origin_(origin),num_x_(num_x),num_y_(num_y),resolution_(resolution)
 {
  unsigned char imgmat[100][100] = {0};
  vector<float> bearing_raw, range_raw;
  for(int i = 0; i<laser_scan->ranges.size(); i++)
  {
   if(laser_scan->ranges[i] < obstacle_range && laser_scan->ranges[i] > min_range)
   {
    range_raw.push_back(laser_scan->ranges[i]);
    bearing_raw.push_back(laser_scan->angle_min+i*laser_scan->angle_increment);  
   }
  }
  for(int j = 0; j<range_raw.size(); j++)    //Traverse each laser beam
  {
   int x_laser = range_raw[j]*::cos(bearing_raw[j])*5 + 50;
   int y_laser = range_raw[j]*::sin(bearing_raw[j])*5 + 50;
   if(x_laser<100&&y_laser<100&&x_laser>=0&&y_laser>=0)
   {
      if(imgmat[x_laser][y_laser] == 0)
      {
       imgmat[x_laser][y_laser] = 1;
       ranges.push_back(range_raw[j]);
       bearings.push_back(bearing_raw[j]);    
      }
   }
  }
  std::vector<bool> seen_tmp(num_x_*num_y_,false);
  seen_ = seen_tmp;
 }
 
SparseScan localScanToIndex(Eigen::Vector3f initial_pose)
{
 SparseScan scans_index;
 float initial_rotation = initial_pose.z();
 double cosAngle,sinAngle;
 double laser_pose_x = laser_pose_.x();
 double laser_pose_y = laser_pose_.y();
 double initial_pose_x = initial_pose.x();
 double initial_pose_y = initial_pose.y();

 int index;
 vector<int> indices;
  for(int i = 0; i < ranges.size(); ++i)
  {
     cosAngle = cos(initial_rotation);
     sinAngle = sin(initial_rotation);
     pair<int,int> tmpIndex = FetchCellIndex
                    (
                     cosAngle*laser_pose_x-sinAngle*laser_pose_y + 
                     ranges[i]*cos(initial_rotation + bearings[i]) + initial_pose_x,
                     sinAngle*laser_pose_x+cosAngle*laser_pose_y + 
                     ranges[i]*sin(initial_rotation + bearings[i]) +initial_pose_y   
                    );         
     index = ToFlatIndex(tmpIndex);
     if(tmpIndex.first >= num_x_ || tmpIndex.second >= num_y_)
     {
      cout<<"index x: "<<tmpIndex.first<<" second y: "<<tmpIndex.second<<endl;
      cout<<"position x: "<<cosAngle*laser_pose_x-sinAngle*laser_pose_y + ranges[i]*cos(initial_rotation + bearings[i])
           <<" position y "<<sinAngle*laser_pose_x+cosAngle*laser_pose_y + ranges[i]*sin(initial_rotation + bearings[i])<<endl;
     }
     assert(tmpIndex.first < num_x_ && tmpIndex.second < num_y_);
     if(count(indices.begin(),indices.end(),index)==0)
     {
      indices.push_back(index);
      scans_index.push_back(tmpIndex);
     }
  }
 return scans_index;
}


vector<tuple<int,int,int> > indexAffected(Eigen::Vector3f initial_pose)
{
  double secs =ros::Time::now().toSec();
 SparseScan scan = localScanToIndex(initial_pose);
 for(int i = 0; i<scan.size(); ++i)
 {
  assert(scan[i].first < num_x_ && scan[i].second < num_y_);
  assert(ToFlatIndex(scan[i])<num_x_ * num_y_);
  indexValue.push_back(forward_as_tuple(scan[i].first,scan[i].second,254));
  inflation_queue[0.0].push_back(CellData(ToFlatIndex(scan[i]),scan[i].first,scan[i].second,scan[i].first,scan[i].second));
  seen_[ToFlatIndex(scan[i])] = true;
 }
 
  std::map<double,vector<CellData> >::iterator queue_iter;
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
 return indexValue;
}

 private:
 sensor_msgs::LaserScanPtr laser_scan;
 Eigen::Vector3f laser_pose_;
 //得到一个点相对于地图原点的x,y索引值
 std::pair<int,int> FetchCellIndex(const float& pt_x, const float& pt_y) const {
     return std::make_pair(
        std::floor((pt_x - origin_.x()) / resolution_) ,
        std::floor((pt_y - origin_.y()) / resolution_)        
         );
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
     if(mx>num_x_ - 1 || my > num_y_ -1) {cout<<"mx: "<<mx<<" my: "<<my<<" num_x: "<<num_x_<<" num_y: "<<num_y_<<endl;}
   assert(mx<=num_x_ - 1 && my <= num_y_ -1);
     indexValue.emplace_back(forward_as_tuple(mx,my,cost));
     seen_[index] = true;
    }
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
 
 
 inline int ToFlatIndex(const pair<int,int> cell_index) const {
    return num_x_ * cell_index.second + cell_index.first;
  }
  
 inline int ToFlatIndex(int cell_index_x, int cell_index_y) const {
    return num_x_ * cell_index_y + cell_index_x;
  }
 std::map<double,vector<CellData> > inflation_queue;
 vector<tuple<int,int,int> > indexValue;
 std::vector<bool> seen_;
 vector<float> bearings;
 vector<float> ranges;
 float width_, height_, resolution_;
 int num_x_, num_y_;
 Eigen::Array2f origin_;
 float cell_inflation_radius_ = 10;//以栅格为单位的膨胀半径，10等于50cm
 float obstacle_range = 1.9;//标记障碍物的距离
 float min_range = 0.05;//最近障碍物距离
};
}

#endif  
