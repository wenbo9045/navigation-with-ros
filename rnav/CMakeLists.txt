cmake_minimum_required(VERSION 2.8.3)
project(rnav)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  std_msgs
  tf
  pcl_conversions
)
find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)
catkin_package(
    CATKIN_DEPENDS
    roscpp
    tf
    INCLUDE_DIRS include
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

 add_library(planner_support  src/local_planner/trajectory.cpp
                              src/local_planner/odometry_helper_ros.cpp
                              src/local_planner/map_cell.cpp
                              src/local_planner/map_grid.cpp)
 add_library(goal_functions src/local_planner/goal_functions.cpp)
 add_library(local_planner  src/local_planner/tri_planner_ros.cpp)
 target_link_libraries(local_planner goal_functions planner_support)
 
 add_library(global_planner  src/global_planner/navfn_ros.cpp
     			     src/global_planner/navfn.cpp)
 add_library(nav_base src/move_base/move_base.cpp)
 add_definitions(${Eigen_DEFINITIONS})
 add_executable(rnav src/rnav_node.cpp)
 add_dependencies(rnav ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(rnav
                       nav_base 
                       local_planner
                       global_planner
                       ${Eigen_LIBRARIES} 
                       ${catkin_LIBRARIES})


