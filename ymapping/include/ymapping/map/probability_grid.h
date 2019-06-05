#ifndef PROBABILITY_GRID_H_
#define PROBABILITY_GRID_H_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include "ymapping/map/probability_values.h"
#include "ymapping/map/map_limits.h"
#include "ymapping/map/xy_index.h"
#include <iostream>
#include <assert.h>
#include <exception>
#include <queue>
#include <tuple>
#include <utility>
#include <functional>
using uint16 = uint16_t;

namespace mapping{
//表示概率的二维网格。
class ProbabilityGrid {
 public:
  explicit ProbabilityGrid(const MapLimits& limits)
      : limits_(limits),
        cells_(limits_.cell_limits().num_x_cells *   //std::vector<uint16> cells_  vector初始化栅格为未知
                   limits_.cell_limits().num_y_cells,
               mapping::kUnknownProbabilityValue),
        map_cells_(limits_.cell_limits().num_x_cells *   //std::vector<signed char> map_cells_  vector初始化栅格为未知
                   limits_.cell_limits().num_y_cells,
               -1),
        cell_value_(limits_.cell_limits().num_x_cells *   //std::vector<int> cell_value_  vector初始化栅格为极限评分
                   limits_.cell_limits().num_y_cells,
               100) {}

  // 返回ProbabilityGrid的界限
  const MapLimits& limits() const { return limits_; }
  

  //结束更新序列。
  void FinishUpdate() {
    while (!update_indices_.empty()) {
      cells_[update_indices_.back()] -= mapping::kUpdateMarker;
      update_indices_.pop_back();
    }
  }
  
  std::vector<int>& update_indices()
  {
   return update_indices_;
  }

  // 将'cell_index'处的单元格概率设置为给定的'概率'。 只有之前cell状态未知时允许使用。
  void SetProbability(const Eigen::Array2i& cell_index,
                      const float probability) {
    uint16& cell = cells_[ToFlatIndex(cell_index)];
    cell = mapping::ProbabilityToValue(probability);
    known_cells_box_.extend(cell_index.matrix());
  }


  // 如果单元格尚未更新，则将调用ComputeLookupTableToApplyOdds（）时指定的'odds'为'cell_index'单元格的概率。
  // 在调用FinishUpdate（）之前，同一单元的多个更新将被忽略。 如果单元格已更新，则返回true。
  // 如果这是对指定单元格的第一次ApplyOdds（）调用，则其值将被设置为对应于“odds”的概率。
  // 如果栅格是未更新的，则cells_中对应的value=0,而table[0]=odds
  bool ApplyLookupTable(const Eigen::Array2i& cell_index,
                        const std::vector<uint16>& table) {
    const int flat_index = ToFlatIndex(cell_index);
    assert(flat_index < cells_.size());
    // 注意是引用，在后面改变了cells_中的值  
   uint16& cell = cells_[flat_index];              

    // 使 hit 优先
    if (cell >= mapping::kUpdateMarker) {
      return false;
    }
    // 类内全局变量更新
    update_indices_.push_back(flat_index);   
    //当前的栅格的value丢进hit_table变成高一些的值
    cell = table[cell];
//    if(mapping::ValueToProbability(cells_[flat_index]) >= 0.89) AddFinishedCell(cell_index.x(),cell_index.y());
    map_cells_[flat_index] = mapping::ValueToProbability(cells_[flat_index])*100;
    known_cells_box_.extend(cell_index.matrix());
    return true;
  }
  
  // Returns the probability of the cell with 'cell_index'.
  //返回单元格'cell_index'的概率
  float GetProbability(const Eigen::Array2i& cell_index) const {
    if (limits_.Contains(cell_index)) {
      return mapping::ValueToProbability(cells_[ToFlatIndex(cell_index)]);
    }
    return mapping::kMinProbability;
  }
  
  float GetProbability(const int x, const int y) const {
    if (limits_.Contains(x,y)) {
    return mapping::ValueToProbability(cells_[limits_.cell_limits().num_x_cells * y + x]);// + exp(-pow(cell_value_[limits_.cell_limits().num_x_cells * y + x],2));
    }
    return mapping::kMinProbability;
  }

  // Returns true if the probability at the specified index is known.
  bool IsKnown(const Eigen::Array2i& cell_index) const {
    return limits_.Contains(cell_index) &&cells_[ToFlatIndex(cell_index)] != mapping::kUnknownProbabilityValue;
  }

  //填充'offset'和'limits'来定义包含所有已知单元格的子区域。
  void ComputeCroppedLimits(Eigen::Array2i* const offset,
                            CellLimits* const limits) const {
    if (known_cells_box_.isEmpty()) {
      *offset = Eigen::Array2i::Zero();
      *limits = CellLimits(1, 1);
    } else {
      *offset = known_cells_box_.min().array();
      *limits = CellLimits(known_cells_box_.sizes().x() + 1,
                           known_cells_box_.sizes().y() + 1);
    }
  }

  //根据需要增加地图以包含“点”。 这改变了这些坐标的含义。 
  // 在对“ApplyLookupTable”进行调用之前，必须在“FinishUpdate”之后立即调用此方法。
  void GrowLimits(const sensor::PointCloud pointCloud) {  
    //首先需要判断传进来的点云与目前坐标系的关系，得到最小和最大的框，此框为新的地图
   float map_min_x = limits_.min().x();
   float map_min_y = limits_.min().y();
   float map_max_x = map_min_x + limits_.cell_limits().num_x_cells*limits_.resolution();
   float map_max_y = map_min_y + limits_.cell_limits().num_y_cells*limits_.resolution();
   //float point_min_x(map_min_x),point_min_y(map_min_y),point_max_x(map_max_x),point_max_y(map_max_y);
   float point_min_x(0),point_min_y(0),point_max_x(0),point_max_y(0);
   //地图原点偏移量
   int offset_x = 0;
   int offset_y = 0;
   for( auto point : pointCloud)
   {
    if(point.x() > point_max_x) point_max_x = point.x();
    if(point.y() > point_max_y) point_max_y = point.y();
    if(point.x() < point_min_x) point_min_x = point.x();
    if(point.y() < point_min_y) point_min_y = point.y();
   }
   //std::cout<<"map_min_x: "<<map_min_x<<" map_max_x: "<<map_max_x<<" map_min_y: "<<map_min_y<<" map_max_y: "<<map_max_y<<std::endl;
   //std::cout<<"point_min_x: "<<point_min_x<<" point_max_x: "<<point_max_x<<" point_min_y: "<<point_min_y<<" point_max_y: "<<point_max_y<<std::endl;std::cout<<std::endl;
   //确定新地图的原点，即最小值的点
   if(point_min_x < map_min_x || point_min_y < map_min_y)
   {
    offset_x = (point_min_x < map_min_x) ? (map_min_x - point_min_x + 1)/limits_.resolution() : 0; 
    offset_y = (point_min_y < map_min_y) ? (map_min_y - point_min_y + 1)/limits_.resolution() : 0; 
    map_min_x = map_min_x - offset_x*limits_.resolution();
    map_min_y = map_min_y - offset_y*limits_.resolution();
   }
   
   //std::cout<<"offset_x: "<<offset_x<<" offset_y: "<<offset_y<<std::endl;
   if(point_max_x < map_max_x && point_max_y < map_max_y && offset_x == 0 && offset_y == 0) return;
   //std::cout<<"get MAX"<<std::endl;
   //确定新地图的顶点，即最大值点
   map_max_x = (point_max_x < map_max_x) ? map_max_x : point_max_x + 1; 
   map_max_y = (point_max_y < map_max_y) ? map_max_y : point_max_y + 1; 
   MapLimits new_limits(limits_.resolution(),limits_.min()-limits_.resolution()*Eigen::Vector2d(offset_x,offset_y),
                        CellLimits(std::ceil(map_max_x - map_min_x)/limits_.resolution(),std::ceil(map_max_y - map_min_y)/limits_.resolution()));
   const int stride = new_limits.cell_limits().num_x_cells;
   const int offset = offset_x + stride * offset_y;
   const int new_size = new_limits.cell_limits().num_x_cells * new_limits.cell_limits().num_y_cells;
   std::vector<uint16> new_cells(new_size, mapping::kUnknownProbabilityValue);
   std::vector<signed char> new_map_cells(new_size, -1);
   std::vector<int> new_cell_value(new_size, 100);
   //std::cout<<"map_min_x: "<<map_min_x<<" map_max_x: "<<map_max_x<<" map_min_y: "<<map_min_y<<" map_max_y: "<<map_max_y<<std::endl;
   //std::cout<<"point_min_x: "<<point_min_x<<" point_max_x: "<<point_max_x<<" point_min_y: "<<point_min_y<<" point_max_y: "<<point_max_y<<std::endl;std::cout<<std::endl;
   //std::cout<<"offset_x: "<<offset_x<<" offset_y: "<<offset_y<<" stride: "<<stride<<" offset: "<<offset<<std::endl;
   //std::cout<<"max old cell index: "<<(offset + limits_.cell_limits().num_x_cells -1 + (limits_.cell_limits().num_y_cells -1)*stride)<<" new_size: "<<new_size<<std::endl;
   //std::cout<<"new x num: "<<new_limits.cell_limits().num_x_cells<<"new y num: "<<new_limits.cell_limits().num_y_cells
   //         <<"old x num: "<<limits_.cell_limits().num_x_cells<<"old y num: "<<limits_.cell_limits().num_y_cells<<std::endl;
   //std::cout<<"ymax: "<<(limits_.cell_limits().num_y_cells -1)*stride <<std::endl;
   assert(offset + limits_.cell_limits().num_x_cells -1 + (limits_.cell_limits().num_y_cells -1)*stride <= new_size);
   for(int i = 0; i < limits_.cell_limits().num_x_cells; ++i)
   {
      for(int j = 0; j < limits_.cell_limits().num_y_cells; ++j)
      {
       new_cells[offset + i + j*stride] = cells_[i+j*limits_.cell_limits().num_x_cells];
       new_map_cells[offset + i + j*stride] = map_cells_[i+j*limits_.cell_limits().num_x_cells];
       new_cell_value[offset + i + j*stride] = new_cell_value[i+j*limits_.cell_limits().num_x_cells];
      }
   }
   limits_ = new_limits;
   cells_ = new_cells;
   map_cells_ = new_map_cells;
   cell_value_ = new_cell_value;
  }
  
  
  //将完全建好的cell放入queue
  void AddFinishedCell(int x, int y) 
  {
   if(cell_value_[ToFlatIndex(std::forward_as_tuple(x,y))] != 0)
   {
    cell_value_[ToFlatIndex(std::forward_as_tuple(x,y))] = 0;
    Queue.push(std::make_tuple(x,y));
    //std::cout<<"Add cell"<<std::endl;
   }
  }
  //对queue里面的所有栅格进行更新
  void ProcessFinishedCell()
  {
   assert(cell_value_.size() == cells_.size());
   assert(map_cells_.size() == cells_.size());
   //std::cout<<"QUEUE SIZE: "<<Queue.size()<<std::endl;
   while(!Queue.empty())
   {
    int x_index = std::get<0>(Queue.front());
    int y_index = std::get<1>(Queue.front());
    int value = cell_value_[ToFlatIndex(std::forward_as_tuple(x_index,y_index))];
    
    if(limits_.Contains(x_index+1,y_index))
    {
     if( cell_value_[ToFlatIndex(std::forward_as_tuple(x_index+1,y_index))] > value + 1)
     {
      cell_value_[ToFlatIndex(std::forward_as_tuple(x_index+1,y_index))] = value + 1;
      if(value < 10)
      Queue.push(std::make_tuple(x_index+1,y_index));
     }
    }
     
    if(limits_.Contains(x_index-1,y_index))
    {
     if( cell_value_[ToFlatIndex(std::forward_as_tuple(x_index-1,y_index))] > value + 1)
     {
      cell_value_[ToFlatIndex(std::forward_as_tuple(x_index-1,y_index))] = value + 1;
      if(value < 10)
      Queue.push(std::make_tuple(x_index-1,y_index));
     }
    }
     
    if(limits_.Contains(x_index,y_index+1))
    {
     if( cell_value_[ToFlatIndex(std::forward_as_tuple(x_index,y_index+1))] > value + 1)
     {
      cell_value_[ToFlatIndex(std::forward_as_tuple(x_index,y_index+1))] = value + 1;
      if(value < 10)
      Queue.push(std::make_tuple(x_index,y_index+1));
     }
    } 
    
    if(limits_.Contains(x_index,y_index-1))
    {
     if( cell_value_[ToFlatIndex(std::forward_as_tuple(x_index,y_index-1))] > value + 1)
     {
      cell_value_[ToFlatIndex(std::forward_as_tuple(x_index,y_index-1))] = value + 1;
      if(value < 10)
      Queue.push(std::make_tuple(x_index,y_index-1));
     }
    }
    Queue.pop();
   }
  }
  
  std::vector<signed char>& map_cell(){return map_cells_;}
  std::vector<uint16> cell(){return cells_;}
  void getLocValue(Eigen::Array2i& index)
  {
   std::cout<<"loc index value: "<<cell_value_[ToFlatIndex(index)]<<"probability value: "<<GetProbability(index)<<std::endl;
  }

 private:

  int ToFlatIndex(const Eigen::Array2i& cell_index) const {
    return limits_.cell_limits().num_x_cells * cell_index.y() + cell_index.x();
  }
  
  int ToFlatIndex(const std::tuple<int,int>& cell_index) const {
    return limits_.cell_limits().num_x_cells * std::get<1>(cell_index) + std::get<0>(cell_index);
  }

  MapLimits limits_;
  std::vector<uint16> cells_;  // Highest bit is update marker. //最高位是更新标记。
  std::vector<int> cell_value_;//用来存储定位所需要的数值
  std::queue<std::tuple<int,int> > Queue;
  std::vector<signed char> map_cells_;   //最高位是更新标记。
  std::vector<int> update_indices_; //已更新栅格的索引
  //已知单元格的边界框可有效计算裁剪限制。
  Eigen::AlignedBox2i known_cells_box_;
};

}  // namespace mapping

#endif  
