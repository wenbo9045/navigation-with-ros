#include "ymapping/transform/rigid_transform.h"
#include "ymapping/map/xy_index.h"
#include "ymapping/map/map_limits.h"
#include "ymapping/map/probability_grid.h"
#include "ros/ros.h"

class ymapping
{
 public:
 ymapping(){}
};
int main(int argc, char** argv)
{
 ros::init(argc,argv,"ymapping");
 ros::spin();
 return 0;
}
