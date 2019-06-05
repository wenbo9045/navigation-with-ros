#include "ymapping/sensor/point_cloud.h"
#include "ymapping/transform/transform.h"

namespace sensor {

PointCloud transformPointCloud(const transform::Rigid2f& trans, const sensor::PointCloud& point_cloud)
{
 PointCloud result;
 result.reserve(point_cloud.size());
 for(auto point : point_cloud)
 {
  result.emplace_back(trans*point);
 }
 return result;
}

void generateRotatedScan(double angle, sensor::PointCloud& PointCloud)
{
 for(int i = 0; i < PointCloud.size(); ++i)
 {
  PointCloud[i] = transform::Rigid2f(transform::Rigid2f::Vector::Zero(),angle)*PointCloud[i];
 }
}

sensor::PointCloud scanToPointCloud(const transform::Rigid2f& trans, const sensor_msgs::LaserScanConstPtr laser_scan)
{
 sensor::PointCloud PCLoud;
 //首先进行点云稀疏化
 unsigned char imgmat[400][400] = {0};
 float increment = laser_scan->angle_increment;
 float angle = laser_scan->angle_min;
 for(int i = 0; i < laser_scan->ranges.size(); ++i)
 {
  if(laser_scan->ranges[i] > 0.05 && laser_scan->ranges[i] < 10)
    PCLoud.push_back(trans*transform::Rigid2f::Vector(laser_scan->ranges[i]*cos(angle),laser_scan->ranges[i]*sin(angle)));
  angle += increment;
 }
 return PCLoud;
}

}  // namespace sensor

