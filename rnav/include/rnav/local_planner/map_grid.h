#ifndef TRI_MAP_GRID_H_
#define TRI_MAP_GRID_H_

#include <vector>
#include <iostream>
#include "map_cell.h"
#include <ros/console.h>
#include <ros/ros.h>
#include "rnav/local_map/local_map.h"
#include <geometry_msgs/PoseStamped.h>

#include "tf/transform_broadcaster.h"  
#include "tf/transform_listener.h"  
#include "tf/tf.h"
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <cmath>
#include <angles/angles.h>
#include <string>
#include <cmath>

#include <angles/angles.h>


namespace tri_local_planner{
  /**
   * @class MapGrid
   * @brief A grid of MapCell cells that is used to propagate path and goal distances for the trajectory controller.
   */
  class MapGrid{
    public:
      
      /**
       * @brief  Creates a 0x0 map by default
       */
      MapGrid();

      /**
       * @brief  Creates a map of size_x by size_y
       * @param size_x The width of the map 
       * @param size_y The height of the map 
       */
      MapGrid(unsigned int size_x, unsigned int size_y);


      /**
       * @brief  Returns a map cell accessed by (col, row)
       * @param x The x coordinate of the cell 
       * @param y The y coordinate of the cell 
       * @return A reference to the desired cell
       */
      inline MapCell& operator() (unsigned int x, unsigned int y){
        return map_[size_x_ * y + x];
      }

      /**
       * @brief  Returns a map cell accessed by (col, row)
       * @param x The x coordinate of the cell 
       * @param y The y coordinate of the cell 
       * @return A copy of the desired cell
       */
      inline MapCell operator() (unsigned int x, unsigned int y) const {
        return map_[size_x_ * y + x];
      }

      inline MapCell& getCell(unsigned int x, unsigned int y){
        return map_[size_x_ * y + x];
      }

      /**
       * @brief  Destructor for a MapGrid
       */
      ~MapGrid(){}

      /**
       * @brief  Copy constructor for a MapGrid
       * @param mg The MapGrid to copy 
       */
      MapGrid(const MapGrid& mg);

      /**
       * @brief  Assignment operator for a MapGrid
       * @param mg The MapGrid to assign from 
       */
      MapGrid& operator= (const MapGrid& mg);

      /**
       * @brief reset path distance fields for all cells
       */
      void resetPathDist();

      /**
       * @brief  check if we need to resize
       * @param size_x The desired width
       * @param size_y The desired height
       */
      void sizeCheck(unsigned int size_x, unsigned int size_y);

      /**
       * @brief Utility to share initialization code across constructors
       */
      void commonInit();

      /**
       * @brief  Returns a 1D index into the MapCell array for a 2D index
       * @param x The desired x coordinate
       * @param y The desired y coordinate
       * @return The associated 1D index 
       */
      size_t getIndex(int x, int y);

      /**
       * return a value that indicates cell is in obstacle
       */
      inline double obstacleCosts() {
        return map_.size();
      }

      /**
       * returns a value indicating cell was not reached by wavefront
       * propagation of set cells. (is behind walls, regarding the region covered by grid)
       */
      inline double unreachableCellCosts() {
        return map_.size() + 1;
      }

      /**
       * @brief  Used to update the distance of a cell in path distance computation
       * @param  current_cell The cell we're currently in 
       * @param  check_cell The cell to be updated
       */
      
       inline bool updatePathCell(MapCell* current_cell, MapCell* check_cell,
       localMap::localMap& costmap);

      /**
       * increase global plan resolution to match that of the costmap by adding points linearly between global plan points
       * This is necessary where global planners produce plans with few points.
       * @param global_plan_in input
       * @param global_plan_output output
       * @param resolution desired distance between waypoints
       */
      static void adjustPlanResolution(const std::vector<geometry_msgs::PoseStamped>& global_plan_in,
            std::vector<geometry_msgs::PoseStamped>& global_plan_out, double resolution);

      /**
       * @brief  Compute the distance from each cell in the local map grid to the planned path
       * @param dist_queue A queue of the initial cells on the path 
       */
      void computeTargetDistance(std::queue<MapCell*>& dist_queue, localMap::localMap& costmap);

      /**
       * @brief Update what cells are considered path based on the global plan 
       */
      void setTargetCells(localMap::localMap& costmap, const std::vector<geometry_msgs::PoseStamped>& global_plan);

      /**
       * @brief Update what cell is considered the next local goal
       */
      void setLocalGoal(localMap::localMap& costmap,
            const std::vector<geometry_msgs::PoseStamped>& global_plan,
            tf::Stamped<tf::Pose> global_pose,double goal_dis);


      double goal_x_, goal_y_; /**< @brief The goal distance was last computed from */

      unsigned int size_x_, size_y_; ///< @brief The dimensions of the grid
      
      
        std::vector<MapCell> map_; ///< @brief Storage for the MapCells
    private:

    
      
     /********************************/
      ros::NodeHandle n_;
      ros::Publisher mapGoalPub_;
      bool pubOK = false;
    /********************************/

  };
};

#endif
