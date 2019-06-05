#include "rnav/local_planner/map_grid.h"
#include <costmap_2d/cost_values.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include "geometry_msgs/PointStamped.h"

using namespace std;

namespace tri_local_planner{

  MapGrid::MapGrid()
    : size_x_(0), size_y_(0)
  {
      mapGoalPub_ = n_.advertise<geometry_msgs::PointStamped>("/localGoal",1);
  }

  MapGrid::MapGrid(unsigned int size_x, unsigned int size_y) 
    : size_x_(size_x), size_y_(size_y)
  {
      mapGoalPub_ = n_.advertise<geometry_msgs::PointStamped>("/localGoal",1);
      commonInit();     
  }

  MapGrid::MapGrid(const MapGrid& mg){
    size_y_ = mg.size_y_;
    size_x_ = mg.size_x_;
    map_ = mg.map_;
    mapGoalPub_ = n_.advertise<geometry_msgs::PointStamped>("/localGoal",1);
  }

  void MapGrid::commonInit(){
    //don't allow construction of zero size grid
    ROS_ASSERT(size_y_ != 0 && size_x_ != 0);

    map_.resize(size_y_ * size_x_);

    //make each cell aware of its location in the grid
    for(unsigned int i = 0; i < size_y_; ++i){
      for(unsigned int j = 0; j < size_x_; ++j){
        unsigned int id = size_x_ * i + j;
        map_[id].cx = j;
        map_[id].cy = i;
      }
    }
  }

  size_t MapGrid::getIndex(int x, int y){
    return size_x_ * y + x;
  }

  MapGrid& MapGrid::operator= (const MapGrid& mg){
    size_y_ = mg.size_y_;
    size_x_ = mg.size_x_;
    map_ = mg.map_;
    return *this;
  }

  void MapGrid::sizeCheck(unsigned int size_x, unsigned int size_y){
    if(map_.size() != size_x * size_y)
      map_.resize(size_x * size_y);

    if(size_x_ != size_x || size_y_ != size_y){
      size_x_ = size_x;
      size_y_ = size_y;

      for(unsigned int i = 0; i < size_y_; ++i){
        for(unsigned int j = 0; j < size_x_; ++j){
          int index = size_x_ * i + j;
          map_[index].cx = j;
          map_[index].cy = i;
        }
      }
    }
  }


  //reset the path_dist and goal_dist fields for all cells
  void MapGrid::resetPathDist(){
    for(unsigned int i = 0; i < map_.size(); ++i) {
      map_[i].target_dist = unreachableCellCosts();
      map_[i].target_mark = false;
      map_[i].within_robot = false;
    }
  }

  void MapGrid::adjustPlanResolution(const std::vector<geometry_msgs::PoseStamped>& global_plan_in,
      std::vector<geometry_msgs::PoseStamped>& global_plan_out, double resolution) {
    if (global_plan_in.size() == 0) {
      return;
    }
    double last_x = global_plan_in[0].pose.position.x;
    double last_y = global_plan_in[0].pose.position.y;
    global_plan_out.push_back(global_plan_in[0]);

    // we can take "holes" in the plan smaller than 2 grid cells (squared = 4)
    double min_sq_resolution = resolution * resolution * 4;

    for (unsigned int i = 1; i < global_plan_in.size(); ++i) {
      double loop_x = global_plan_in[i].pose.position.x;
      double loop_y = global_plan_in[i].pose.position.y;
      double sqdist = (loop_x - last_x) * (loop_x - last_x) + (loop_y - last_y) * (loop_y - last_y);
      if (sqdist > min_sq_resolution) {
        int steps = ((sqrt(sqdist) - sqrt(min_sq_resolution)) / resolution) - 1;
        // add a points in-between
        double deltax = (loop_x - last_x) / steps;
        double deltay = (loop_y - last_y) / steps;
        // TODO: Interpolate orientation
        for (int j = 1; j < steps; ++j) {
          geometry_msgs::PoseStamped pose;
          pose.pose.position.x = last_x + j * deltax;
          pose.pose.position.y = last_y + j * deltay;
          pose.pose.position.z = global_plan_in[i].pose.position.z;
          pose.pose.orientation = global_plan_in[i].pose.orientation;
          pose.header = global_plan_in[i].header;
          global_plan_out.push_back(pose);
        }
      }
      global_plan_out.push_back(global_plan_in[i]);
      last_x = loop_x;
      last_y = loop_y;
    }
  }

  //update what map cells are considered path based on the global_plan
  void MapGrid::setTargetCells(localMap::localMap& costmap,
      const std::vector<geometry_msgs::PoseStamped>& global_plan) {
    sizeCheck(costmap.getSizeInCellsX(), costmap.getSizeInCellsY());

    bool started_path = false;

    queue<MapCell*> path_dist_queue;

    std::vector<geometry_msgs::PoseStamped> adjusted_global_plan;
    adjustPlanResolution(global_plan, adjusted_global_plan, costmap.getResolution());
    if (adjusted_global_plan.size() != global_plan.size()) {
      ROS_DEBUG("Adjusted global plan resolution, added %zu points", adjusted_global_plan.size() - global_plan.size());
    }
  
    unsigned int i;
    // put global path points into local map until we reach the border of the local map
    for (i = 0; i < adjusted_global_plan.size(); ++i) {
      double g_x = adjusted_global_plan[i].pose.position.x;
      double g_y = adjusted_global_plan[i].pose.position.y;
      unsigned int map_x, map_y;
      if (costmap.worldToMap(g_x, g_y, map_x, map_y) && costmap.getCost(map_x, map_y) != costmap_2d::NO_INFORMATION) {
        MapCell& current = getCell(map_x, map_y);
        current.target_dist = 0.0;
        current.target_mark = true;
        path_dist_queue.push(&current);
        started_path = true;
      } else if (started_path) {
          break;
      }
    }
    if (!started_path) {
      ROS_ERROR("None of the %d first of %zu (%zu) points of the global plan were in the local costmap and free",
          i, adjusted_global_plan.size(), global_plan.size());
      return;
    }

    computeTargetDistance(path_dist_queue, costmap);
  }

  void MapGrid::setLocalGoal(localMap::localMap& costmap,
      const std::vector<geometry_msgs::PoseStamped>& global_plan, tf::Stamped<tf::Pose> global_pose,double goal_dis) {
    double global_pose_x = global_pose.getOrigin().x();
	double global_pose_y = global_pose.getOrigin().y();
    sizeCheck(costmap.getSizeInCellsX(), costmap.getSizeInCellsY());

    int local_goal_x = -1;
    int local_goal_y = -1;
    /*************************/
    double global_goal_x = -1;
    double global_goal_y = -1;
    /*************************/
    bool started_path = false;

    std::vector<geometry_msgs::PoseStamped> adjusted_global_plan;
    adjustPlanResolution(global_plan, adjusted_global_plan, costmap.getResolution());
 

    // skip global path points until we reach the border of the local map
    for (unsigned int i = 0; i < adjusted_global_plan.size(); ++i) {
      double g_x = adjusted_global_plan[i].pose.position.x;
      double g_y = adjusted_global_plan[i].pose.position.y;
      unsigned int map_x, map_y;
      if (costmap.worldToMap(g_x, g_y, map_x, map_y) && costmap.getCost(map_x, map_y) != costmap_2d::NO_INFORMATION) {
        local_goal_x = map_x;
        local_goal_y = map_y;
/******************************************************/
        global_goal_x = g_x;
        global_goal_y = g_y;
        started_path = true;
/********************************************************/
      } else {
        if (started_path) {
          break;
        }// else we might have a non pruned path, so we just continue
      }
      //double goal_dis;    
      // goal_dis=0.64;
      
    if(((g_x - global_pose_x)*(g_x - global_pose_x)+(g_y - global_pose_y)*(g_y - global_pose_y)) > goal_dis) break;  //0.4-0.64
    }
    if (!started_path) {    
      ROS_ERROR("None of the points of the global plan were in the local costmap, global plan points too far from robot");
      return;
    }
    queue<MapCell*> path_dist_queue;
    if (local_goal_x >= 0 && local_goal_y >= 0) {
      MapCell& current = getCell(local_goal_x, local_goal_y);
      costmap.mapToWorld(local_goal_x, local_goal_y, goal_x_, goal_y_);
      current.target_dist = 0.0;
      current.target_mark = true;
      path_dist_queue.push(&current);
    }
    /*************************************/
    geometry_msgs::PointStamped localGoal;
    localGoal.header = global_plan[0].header;
    localGoal.point.x = global_goal_x;
    localGoal.point.y = global_goal_y;
   // std::cout<<"local x: "<<global_goal_x<<"local y: "<<global_goal_y<<std::endl;
   if(pubOK)
    mapGoalPub_.publish(localGoal);  
    pubOK = !pubOK;
    /*************************************/
    computeTargetDistance(path_dist_queue, costmap);
  }



  void MapGrid::computeTargetDistance(queue<MapCell*>& dist_queue, localMap::localMap& costmap){
    MapCell* current_cell;
    MapCell* check_cell;
    unsigned int last_col = size_x_ - 1;
    unsigned int last_row = size_y_ - 1;
    while(!dist_queue.empty()){
      current_cell = dist_queue.front();


      dist_queue.pop();

      if(current_cell->cx > 0){
        check_cell = current_cell - 1;
        if(!check_cell->target_mark){
          //mark the cell as visisted
          check_cell->target_mark = true;
          if(updatePathCell(current_cell, check_cell, costmap)) {
            dist_queue.push(check_cell);
          }
        }
      }

      if(current_cell->cx < last_col){
        check_cell = current_cell + 1;
        if(!check_cell->target_mark){
          check_cell->target_mark = true;
          if(updatePathCell(current_cell, check_cell, costmap)) {
            dist_queue.push(check_cell);
          }
        }
      }

      if(current_cell->cy > 0){
        check_cell = current_cell - size_x_;
        if(!check_cell->target_mark){
          check_cell->target_mark = true;
          if(updatePathCell(current_cell, check_cell, costmap)) {
            dist_queue.push(check_cell);
          }
        }
      }

      if(current_cell->cy < last_row){
        check_cell = current_cell + size_x_;
        if(!check_cell->target_mark){
          check_cell->target_mark = true;
          if(updatePathCell(current_cell, check_cell, costmap)) {
            dist_queue.push(check_cell);
          }
        }
      }
    }
  }

};


  inline bool MapGrid::updatePathCell(MapCell* current_cell, MapCell* check_cell,
      localMap::localMap& costmap){

    //if the cell is an obstacle set the max path distance
    unsigned char cost = costmap.getCost(check_cell->cx, check_cell->cy);
    if(! getCell(check_cell->cx, check_cell->cy).within_robot &&
        (cost == localMap::LETHAL_OBSTACLE ||
         cost == localMap::INSCRIBED_INFLATED_OBSTACLE ||
         cost == localMap::NO_INFORMATION)){
      check_cell->target_dist = obstacleCosts();
      return false;
    }

    double new_target_dist = current_cell->target_dist + 1;
    if (new_target_dist < check_cell->target_dist) {
      check_cell->target_dist = new_target_dist;
    }
    return true;
  }
