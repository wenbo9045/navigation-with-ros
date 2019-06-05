#ifndef GLOBAL_POINT_H_
#define GLOBAL_POINT_H_

#include <pcl/register_point_struct.h>

namespace navfn {
    struct PotarrPoint {
        float x;
        float y;
        float z;
        float pot_value;
    };
}

POINT_CLOUD_REGISTER_POINT_STRUCT(
        navfn::PotarrPoint,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, pot_value, pot_value));

#endif

