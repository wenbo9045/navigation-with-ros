#ifndef PF_H
#define PF_H

#include <map>
#include "pf_vector.h"
#include <tuple>
#include <vector>
#include "boost/shared_ptr.hpp"
#include "sensor_msgs/LaserScan.h"
#include "bmcl/map/map.h"
#include <utility>
#include "Eigen/Core"
#include "Eigen/Geometry"
using namespace std;
typedef std::vector<pair<int,int> > SparseScan;
struct point
{
 double x;
 double y;
};

struct line
{
 point start;
 point end;
 int property_value;
};

struct sample
{
 pf_vector_t pose;
 double average_error;
 int bingo_count;
};

struct sample_cluster
{
 double weight;
 int count;
 pf_vector_t mean;
 pf_matrix_t cov; 
};

class pFilter
{
 public:
 pFilter(map_t* map_);

/*********************Submap移植******************************/
void SetInfo(const sensor_msgs::LaserScanConstPtr& laser_scan, const Eigen::Vector3f laser_pose);
vector<SparseScan> GenerateRotatedScans(Eigen::Vector3f initial_pose_estimate, float angle, float delta);
tuple<int,int,int> scoreCandidates(vector<SparseScan>& scans, float& score);
/*************************************************************/
 /*external access to the sample set*/
 std::vector<sample> sample_set;
 std::vector<sample> auxillary_sample_set;
 pf_vector_t pf_mean;
 vector<line> map_line;
 int lineSelected;
 bool ifselectLine;
 private:
 vector<float> ranges;
 vector<float> bearings;
 float linear_vel_;
 float angular_vel_;
 float angular_vel_max = 0;
 bool angular_plus = true;
 std::vector<boost::shared_ptr<sample> > sample_cluster;
 std::vector<point> submap_point;
 multimap<double,sample> sContainer; 
 float uncertaintyCounter;
 int trust_counter;
 int lineCounter;
 int pf_max_number;
 int pf_min_number;
 pf_matrix_t pf_cov;
 map_t* processed_map;
 bool ifincorridor;
 
 Eigen::Vector3f laser_pose_;
};

#endif 
