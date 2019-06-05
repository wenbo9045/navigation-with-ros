#include "ymapping/map/map_limits.h"
#include "ymapping/map/probability_grid.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
#include <utility>
#include <tuple>
using namespace std;
typedef std::vector<Eigen::Array2i> DiscreteScan;
typedef std::vector<pair<int,int> > SparseScan;
struct Candidate {
  Candidate(float candidate_x,float candidate_y,float candidate_angle)
      : x(candidate_x),y(candidate_y),angle(candidate_angle) {}


  double x = 0.;
  double y = 0.;
  double angle = 0.;

  // Score, higher is better.
  float score = 0.f;

  bool operator<(const Candidate& other) const { return score < other.score; }
  bool operator>(const Candidate& other) const { return score > other.score; }
};
 
class Ploc
{
 public:
 Ploc(const sensor_msgs::LaserScanConstPtr& laser_scan, const Eigen::Vector3f laser_pose, mapping::ProbabilityGrid* probability_grid);
vector<SparseScan> GenerateRotatedScans(Eigen::Vector3f initial_pose_estimate, float angle, float delta);
float scoreCandidates(vector<DiscreteScan>& scans);
tuple<int,int,int> scoreCandidates(vector<SparseScan>& scans, float& score);
 vector<DiscreteScan> TranslateScans(vector<DiscreteScan>& scans, Eigen::Array2i index);
 std::vector<Candidate> candidates;
 private:
 vector<float> ranges;
 vector<float> bearings;
 const mapping::ProbabilityGrid* pGrid;
 const Eigen::Vector3f laser_pose_;
};
