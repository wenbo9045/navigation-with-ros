#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include "Eigen/Geometry"
#include "ymapping/map/submaps.h"
#include "ros/ros.h"
#include <iostream>
namespace mapping{
Submap::Submap(const MapLimits& limits, const transform::Rigid2f& origin)
    : origin_(origin),
      probability_grid_(limits) 
{}

Submap::Submap(const MapLimits& limits, const transform::Rigid2f& origin,const float& hitProbability, const float& missProbability)
    : origin_(origin),
      probability_grid_(limits),
      hit_table_(mapping::ComputeLookupTableToApplyOdds(mapping::Odds(hitProbability))),
      miss_table_(mapping::ComputeLookupTableToApplyOdds(mapping::Odds(missProbability)))
{}

void Submap::InsertRangeData(const sensor::PointCloud& point_cloud,const Eigen::Vector2f& origin)
{
 mapping::MapLimits limits = probability_grid_.limits();
 //首先利用hit table表示命中的栅格，然后用miss table更新未命中的栅格
 for(const Eigen::Vector2f& hit : point_cloud)
 {
  if(limits.Contains(limits.GetCellIndex(hit)))
  {
   if(CheckAndInsert(limits.GetCellIndex(origin).x(),limits.GetCellIndex(origin).y(),limits.GetCellIndex(hit).x(),limits.GetCellIndex(hit).y()))
   {
    probability_grid_.ApplyLookupTable(limits.GetCellIndex(hit),hit_table_);
    Bresenham(limits.GetCellIndex(origin).x(),limits.GetCellIndex(origin).y(),limits.GetCellIndex(hit).x(),limits.GetCellIndex(hit).y());
   }
  }
  else
  {
  ROS_ERROR("SHIT HAPPENS!!! x: %f, y: %f, x_index: %d, y_index:%d, x_limit: %d y_limit: %d",hit.x(),hit.y(),
                                                                                             limits.GetCellIndex(hit).x(),
                                                                                             limits.GetCellIndex(hit).y(),
                                                                                             limits.cell_limits().num_x_cells,
                                                                                             limits.cell_limits().num_y_cells
                                                                                             );
  }
 }
}

bool Submap::CheckAndInsert(int xx1, int yy1, int xx2, int yy2)
{
  //定义差量，误差等变量
   int dx, dy, a, a1, e;
   int x, y;
   dx = abs (xx2 - xx1);
   dy = abs (yy2 - yy1);
    /* 设定起始点 */    
   y = yy1;
   x = xx1;
   //表明方向的变量
   int sx, sy;

   if (xx1 > xx2) sx = -1;//x递减方向
   else sx = 1;//x递增方向

   if (yy1 > yy2) sy = -1;//y递减方向
   else sy = 1;//y递增方向

   if (dx >= dy)
    { e=0;
      for (x = xx1; (sx >= 0 ? x <= xx2 : x >= xx2); x += sx)
        {
          if(probability_grid_.GetProbability(x,y) >= 0.89) return false;//{probability_grid_.AddFinishedCell(x,y);return false;}
          e+=dy;
          if ((e<<1) >= dx){y += sy;e -= dx;}   
        }
    }
   else
   {  e=0;
      for (y = yy1; (sy >= 0 ? y <= yy2 : y >= yy2); y += sy)
        {
          if(probability_grid_.GetProbability(x,y) >= 0.89) return false;//{probability_grid_.AddFinishedCell(x,y);return false;}
          e+=dx;
          if ((e<<1) >= dy) {x += sx;e -= dy;}    
        }
   }
 return true;
}

//先用bresenham,如果碰到完全建完的栅格，则退出
void Submap::Bresenham(int xx1, int yy1, int xx2, int yy2)
{
   int dx, dy, a, a1, e;//定义差量，误差等变量
   int x, y;
   dx = abs (xx2 - xx1);
   dy = abs (yy2 - yy1);
    /* 设定起始点 */    
   y = yy1;
   x = xx1;
   int sx, sy;//表明方向的变量

   if (xx1 > xx2) sx = -1;//x递减方向
   else sx = 1;//x递增方向

   if (yy1 > yy2) sy = -1;//y递减方向
   else sy = 1;//y递增方向

   if (dx >= dy)
    { e=0;
      for (x = xx1; (sx >= 0 ? x <= xx2 : x >= xx2); x += sx)
        {
//          if(probability_grid_.GetProbability(x,y) == 0.9) return;
          probability_grid_.ApplyLookupTable(Eigen::Array2i(x,y),miss_table_);
          e+=dy;
          if ((e<<1) >= dx){y += sy;e -= dx;}   
        }
    }
   else
   {  e=0;
      for (y = yy1; (sy >= 0 ? y <= yy2 : y >= yy2); y += sy)
        {
//          if(probability_grid_.GetProbability(x,y) == 0.9) return;
          probability_grid_.ApplyLookupTable(Eigen::Array2i(x,y),miss_table_);
          e+=dx;
          if ((e<<1) >= dy) {x += sx;e -= dy;}    
        }
   }
}

void Submap::MapCheck(const sensor::PointCloud& PCLoud){
 probability_grid_.GrowLimits(PCLoud);
}

void Submap::Finish() {
  probability_grid_.FinishUpdate();
  probability_grid_.ProcessFinishedCell();
  updated_ = true;
}

}  // namespace mapping_2d
