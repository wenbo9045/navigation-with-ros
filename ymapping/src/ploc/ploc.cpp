#include "ymapping/ploc/ploc.h"
#include <map>
#include <tuple>
using namespace std;
Ploc::Ploc(const sensor_msgs::LaserScanConstPtr& laser_scan, const Eigen::Vector3f laser_pose, mapping::ProbabilityGrid* probability_grid):
          pGrid(probability_grid),laser_pose_(laser_pose)
{
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
       ranges.push_back(range_raw[j]);
       bearings.push_back(bearing_raw[j]);      
      }
   }
  }
};


tuple<int,int,int> Ploc::scoreCandidates(vector<SparseScan>& scans,float& score)
{
 int size0 = scans.size();
 int size1 = scans[0].size();
 float value;
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
     value += pGrid->GetProbability(scans[scan_num][point_num].first + x_offset,scans[scan_num][point_num].second + y_offset);//*std::exp(-std::pow(std::hypot(x_offset,y_offset),2)/20);
    }
    candidates.emplace(value,forward_as_tuple(x_offset,y_offset,scan_num));
   }
  }
 }
 
 multimap<float,tuple<int,int,int> >::iterator it = candidates.end();
 it--;
 int x,y,th;
 tuple<int,int,int> ret = it->second;
 score = it->first;
 for(int i = 0; i < 30; ++i) {it--;}
 score = score - it->first;
 //std::cout<<"size: "<<candidates.size()<<" best candidates: "<<x<<" "<<y<<" "<<th<<std::endl;
 return (ret);
}


 vector<DiscreteScan> Ploc::TranslateScans(vector<DiscreteScan>& scans, Eigen::Array2i index)
 {
  vector<DiscreteScan> scans_index;
  scans_index.reserve(scans.size());
  int size0 = scans.size();
  int size1 = scans[0].size();
  float value;
 for(int i = 0; i<size0; ++i)
  {
   scans_index.emplace_back();
   scans_index.back().reserve(size1);
   for(int j = 0; j<size1; ++j);
    value = pGrid->GetProbability(1,2);
  } 
  return scans_index;
 }


vector<SparseScan> Ploc::GenerateRotatedScans(Eigen::Vector3f initial_pose_estimate, float angle, float delta)
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
   //scan_index.emplace_back(pGrid->limits().GetCellIndex
     scans_index.back().push_back(pGrid->limits().FetchCellIndex
                    (
                     cosAngle*laser_pose_x-sinAngle*laser_pose_y + 
                     ranges[i]*cos(initial_rotation + orientation + bearings[i]) + initial_pose_estimate_x,
                     sinAngle*laser_pose_x+cosAngle*laser_pose_y + 
                     ranges[i]*sin(initial_rotation + orientation + bearings[i]) +initial_pose_estimate_y   
                    )
                    );
  }
  //scans_index.push_back(scan_index);
 }
 return scans_index;
}


 
 

