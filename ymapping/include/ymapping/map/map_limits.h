#ifndef MAP_LIMITS_H_
#define MAP_LIMITS_H_

#include <utility>
#include <vector>
#include <cmath>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ymapping/map/xy_index.h"
#include "ymapping/sensor/point_cloud.h"
#include "ymapping/transform/rigid_transform.h"
#include "ymapping/transform/transform.h"
#include <iostream>
namespace mapping{

// Defines the limits of a grid map. This class must remain inlined for
// performance reasons.
  //定义网格图的范围。 出于性能原因，此类必须保持内联。
class MapLimits {
 public:
  MapLimits(const double resolution, const Eigen::Vector2d& min,
            const CellLimits& cell_limits)
      : resolution_(resolution), min_(min), cell_limits_(cell_limits) {
  }

  //返回分辨率
  double resolution() const { return resolution_; }
  //网络最大顶点
  const Eigen::Vector2d& min() const { return min_; }     

  //返回网络中X和Y轴上有多少个格。
  const CellLimits& cell_limits() const { return cell_limits_; }


  //返回包含的“点”的单元格的索引，点可能在地图之外，即，对于负值或太大的索引，Contains（）将返回false。
  //地图的远点在左下角(0,0)处
  Eigen::Array2i GetCellIndex(const Eigen::Vector2f& point) const {
    return Eigen::Array2i(
       std::floor((point.x() - min_.x()) / resolution_ + 0.5) ,
       std::floor((point.y() - min_.y()) / resolution_ + 0.5)        
        );
  }

  std::pair<int,int> FetchCellIndex(const float& pt_x, const float& pt_y) const {
    return std::make_pair(
       std::floor((pt_x - min_.x()) / resolution_ + 0.5) ,
       std::floor((pt_y - min_.y()) / resolution_ + 0.5)        
        );
  }
  // Returns true if the ProbabilityGrid contains 'cell_index'.
  bool Contains(const Eigen::Array2i& cell_index) const {
    return (Eigen::Array2i(0, 0) <= cell_index).all() &&
           (cell_index < Eigen::Array2i(cell_limits_.num_x_cells, cell_limits_.num_y_cells)).all();
  }
  
  bool Contains(const int x, const int y) const {
    return (x>=0&&y>=0&&x<cell_limits_.num_x_cells&&y<cell_limits_.num_y_cells);
  }

 private:
  double resolution_;//0.05
  Eigen::Vector2d min_;//(-1,-1)
  CellLimits cell_limits_;
};

}  // namespace mapping_2d

#endif  // CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_
