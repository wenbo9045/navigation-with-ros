#ifndef CARTOGRAPHER_MAPPING_2D_SUBMAPS_H_
#define CARTOGRAPHER_MAPPING_2D_SUBMAPS_H_

#include <memory>
#include <vector>

#include "Eigen/Core"

#include "ymapping/map/map_limits.h"
#include "ymapping/map/probability_grid.h"
#include "ymapping/sensor/point_cloud.h"
#include "ymapping/transform/rigid_transform.h"


namespace mapping{

class Submap{
 public:
  Submap(const MapLimits& limits, const transform::Rigid2f& origin);
  Submap(const MapLimits& limits, const transform::Rigid2f& origin, const float& hitProbability, const float& minssProbability);
  ProbabilityGrid& probability_grid() { return probability_grid_; }
  transform::Rigid2f getOrigin() const {return origin_;}
  void InsertRangeData(const sensor::PointCloud& point_cloud,const Eigen::Vector2f& origin);
  bool CheckAndInsert(int xx1, int yy1, int xx2, int yy2);
  void Bresenham(int xx1, int yy1, int xx2, int yy2);
  void MapCheck(const sensor::PointCloud& PCLoud);
  void Finish();
  bool SubmapStatus() {return updated_;}
  void InitializeSubmap() {updated_ = true;}
 private:
  ProbabilityGrid probability_grid_;
  bool updated_ = false;
  const transform::Rigid2f origin_;
  std::vector<uint16> hit_table_;
  std::vector<uint16> miss_table_; 
};

}  // namespace mapping_2d
#endif  // CARTOGRAPHER_MAPPING_2D_SUBMAPS_H_
