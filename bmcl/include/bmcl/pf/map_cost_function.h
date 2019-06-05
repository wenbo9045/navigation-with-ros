#ifndef MAP_COSTFUNCIION_H
#define MAP_COSTFUNCTION_H
#include <iostream>
#include "bmcl/map/map.h"
#include <math.h>
#include "pf.h"
#include "pf_vector.h"
#include <stdlib.h>
#include <vector>
using namespace std;
class MapCostFunction
{
public:
MapCostFunction(map_t* map):map_(map)
{
 
}

MapCostFunction(map_t* map,
		vector<double> bearing,
		vector<double> range,
		pf_vector_t laser_pose_vector):
		map_(map),bearing_(bearing),range_(range),laser_pose_vector_(laser_pose_vector)
{
 
}

static ceres::CostFunction* CreateAutoDiffCostFunction(map_t* map, 
						       vector<double> bearing, 
						       vector<double> range,
						       pf_vector_t laser_pose_vector) 
{
    return new ceres::AutoDiffCostFunction<MapCostFunction,ceres::DYNAMIC ,3 >
    (
        new MapCostFunction(map,bearing,range,laser_pose_vector),
        bearing.size()
    );
}

template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    map_grid a(map_);
    ceres::BiCubicInterpolator<map_grid> interpolator(a);
    T laserBaseOnSample[2];
    laserBaseOnSample[0] = pose[0] + laser_pose_vector_.v[0]*cos(pose[2]) - laser_pose_vector_.v[1]*sin(pose[2]);
    laserBaseOnSample[1] = pose[1] + laser_pose_vector_.v[0]*sin(pose[2]) + laser_pose_vector_.v[1]*cos(pose[2]);
    T sampleYaw = pose[2];   
    for (size_t j = 0; j < range_.size(); ++j) 
    {
     T angle = sampleYaw+bearing_[j];
     T x_world = laserBaseOnSample[0]+T(range_[j])*cos(angle);
     T y_world = laserBaseOnSample[1]+T(range_[j])*sin(angle);
     T mIndex_x = (x_world - T(map_->origin_x)) / T(map_->scale) - T(0.5)+ T(map_->size_x / 2);
     T mIndex_y = (y_world - T(map_->origin_y)) / T(map_->scale) - T(0.5)+ T(map_->size_y / 2);
     T f;
     interpolator.Evaluate(mIndex_y,mIndex_x,&f); 
     residual[j] = 1. - f;    
    }
    return true;
  }

float retrieveValue(double row, double column)
{
 map_grid a(map_);
 ceres::BiCubicInterpolator<map_grid> interpolator(a);
 double f;
 interpolator.Evaluate(row,column, &f);
 return f; 
}
private:
map_t* map_;
vector<double> bearing_;
vector<double> range_;
pf_vector_t laser_pose_vector_;
 class map_grid
 {
  public:
  map_grid(map_t* map):mapGet_(map){}
  enum { DATA_DIMENSION = 1 };
  void GetValue(const int row, const int column, double* value) const
  {
   if(MAP_VALID(mapGet_, column, row))
   {
    float dist = mapGet_->cells[MAP_INDEX(mapGet_,column,row)].occ_dist;
    /*float tmp = exp(-100*dist*dist) - 0.1;
    if(tmp<0.1) *value = 0.1;
    else *value = tmp;*/
    if(dist<=0.05) *value = 0.9;
    else *value = 0.1;
  //  cout<<tmp<<endl;
   }
   else
   *value = 0.1;
  }
  private:
  map_t* mapGet_;
 };
};
#endif
