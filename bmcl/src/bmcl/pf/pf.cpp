#include <assert.h>
#include <stdlib.h>
#include <time.h>
#include <float.h>
#include <assert.h>
#include <map>
#include "bmcl/pf/pf.h"
#include "ros/ros.h"
using namespace std;
pFilter::pFilter(map_t* map_):
 		 processed_map(map_)
{

}


void pFilter::SetInfo(const sensor_msgs::LaserScanConstPtr& laser_scan, const Eigen::Vector3f laser_pose)
{
  vector<float> rangesTmp;
  vector<float> bearingsTmp;
  laser_pose_ = laser_pose;
  unsigned char imgmat[400][400] = {0};
  vector<float> bearing_raw, range_raw;
  for(int i = 0; i<laser_scan->ranges.size(); i++)
  {
   range_raw.push_back(laser_scan->ranges[i]);
   bearing_raw.push_back(laser_scan->angle_min+i*laser_scan->angle_increment);
  }
  for(int j = 0; j<range_raw.size(); j++)    //Traverse each laser beam
  {
   int x_laser = range_raw[j]*::cos(bearing_raw[j])*20 + 200;
   int y_laser = range_raw[j]*::sin(bearing_raw[j])*20 + 200;
   if(x_laser<400&&y_laser<400&&x_laser>=0&&y_laser>=0)
   {
      if(imgmat[x_laser][y_laser] == 0)
      {
       imgmat[x_laser][y_laser] = 1;
       rangesTmp.push_back(range_raw[j]);
       bearingsTmp.push_back(bearing_raw[j]);      
      }
   }
  }
  ranges = rangesTmp;
  bearings = bearingsTmp;
}

vector<SparseScan> pFilter::GenerateRotatedScans(Eigen::Vector3f initial_pose_estimate, float angle, float delta)
{
 vector<SparseScan> scans_index;
 float initial_rotation = initial_pose_estimate.z();
 scans_index.reserve(2*angle/delta + 1);
 double cosAngle,sinAngle;
 double laser_pose_x = laser_pose_.x();
 double laser_pose_y = laser_pose_.y();
 double initial_pose_estimate_x = initial_pose_estimate.x();
 double initial_pose_estimate_y = initial_pose_estimate.y();
 for(float angle_index = -angle; angle_index <= angle; angle_index += delta)
 {
  float orientation = angle_index * 3.14159/180;
  //DiscreteScan scan_index;
  scans_index.emplace_back();
  scans_index.back().reserve(ranges.size());
  for(int i = 0; i < ranges.size(); ++i)
  {
     cosAngle = cos(initial_rotation + orientation);
     sinAngle = sin(initial_rotation + orientation);
     scans_index.back().push_back(std::make_pair(
                                                 MAP_GXWX(processed_map,cosAngle*laser_pose_x-sinAngle*laser_pose_y + 
                                                 ranges[i]*cos(initial_rotation + orientation + bearings[i]) + initial_pose_estimate_x),
                                                 MAP_GYWY(processed_map,sinAngle*laser_pose_x+cosAngle*laser_pose_y + 
                                                 ranges[i]*sin(initial_rotation + orientation + bearings[i]) +initial_pose_estimate_y )
                                                )
                                );
  }
 }
 return scans_index;
}

tuple<int,int,int> pFilter::scoreCandidates(vector<SparseScan>& scans,float& score)
{
 int size0 = scans.size();
 int size1 = scans[0].size();
 float value;
 float z;
 int mIndex_x,mIndex_y;
 multimap<float,tuple<int,int,int> > candidates;
 for(int x_offset = -2; x_offset <= 2; ++x_offset)
 {
  for(int y_offset = -2; y_offset <= 2; ++y_offset)
  {
   for(int scan_num = 0; scan_num < size0; ++scan_num)
   {
    value = 0;
    for(int point_num = 0; point_num < size1; ++point_num)
    {
     mIndex_x = scans[scan_num][point_num].first + x_offset;
     mIndex_y = scans[scan_num][point_num].second + y_offset;
     if(!MAP_VALID(processed_map, mIndex_x, mIndex_y))
     z = processed_map->max_occ_dist;
     else
     z = processed_map->cells[MAP_INDEX(processed_map,mIndex_x, mIndex_y)].occ_dist;
     //value += exp(-z); 
     value += z;
    }
    candidates.emplace(value,forward_as_tuple(x_offset,y_offset,scan_num));
   }
  }
 }
 
 //multimap<float,tuple<int,int,int> >::iterator it = candidates.end();
 //it--;
 multimap<float,tuple<int,int,int> >::iterator it = candidates.begin();
 int x,y,th;
 tuple<int,int,int> ret = it->second;
 score = it->first;
 for(int i = 0; i < 30; ++i) {it--;}
 score = score - it->first;
 return (ret);
}
