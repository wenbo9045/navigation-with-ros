#ifndef POINT_CLOUD_H_
#define POINT_CLOUD_H_
#include <vector>

#include "Eigen/Core"
#include "ymapping/transform/rigid_transform.h"
#include "sensor_msgs/LaserScan.h"

namespace sensor {

// Stores 2D positions of points.
typedef std::vector<Eigen::Vector2f> PointCloud;

PointCloud transformPointCloud(const transform::Rigid2f& trans, const sensor::PointCloud& point_cloud);
void generateRotatedScan(double angle, sensor::PointCloud& PointCloud);
sensor::PointCloud scanToPointCloud(const transform::Rigid2f& trans, const sensor_msgs::LaserScanConstPtr laser_scan);

}  // namespace sensor

#endif  
