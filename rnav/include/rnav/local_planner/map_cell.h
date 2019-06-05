#ifndef TRI_MAP_CELL_H_
#define TRI_MAP_CELL_H_

#include <limits>

#ifndef DBL_MAX   /* Max decimal value of a double */
#define DBL_MAX   std::numeric_limits<double>::max()  // 1.7976931348623157e+308
#endif

#ifndef DBL_MIN //Min decimal value of a double
#define DBL_MIN   std::numeric_limits<double>::min()  // 2.2250738585072014e-308
#endif

namespace tri_local_planner {
  /**
   * @class MapCell
   * @brief Stores path distance and goal distance information used for scoring trajectories
   */
  class MapCell{
    public:
      /**
       * @brief  Default constructor
       */
      MapCell();

      /**
       * @brief  Copy constructor
       * @param mc The MapCell to be copied
       */
      MapCell(const MapCell& mc);

      unsigned int cx, cy; ///< @brief Cell index in the grid map

      double target_dist; ///< @brief Distance to planner's path

      bool target_mark; ///< @brief Marks for computing path/goal distances

      bool within_robot; ///< @brief Mark for cells within the robot footprint
  };
};

#endif
