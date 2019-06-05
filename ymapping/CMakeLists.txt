cmake_minimum_required(VERSION 2.8.3)
project(ymapping)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
            cmake_modules
            tf
            roscpp
)
find_package(Eigen REQUIRED)
FIND_PACKAGE( OpenCV REQUIRED )
set(CMAKE_CXX_FLAGS "-std=c++11")

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
)

add_definitions(${Eigen_DEFINITIONS})

add_library(transform src/transform/rigid_transform.cpp)
add_library(sensor src/sensor/point_cloud.cpp)
add_library(map       src/map/probability_values.cpp
                      src/map/submaps.cpp
           )
    
add_library(ploc src/ploc/ploc.cpp)  

add_executable(mapping_test src/mapping_test.cpp)
add_dependencies(mapping_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
                      mapping_test
                      sensor
                      map 
                      transform
                      ${Eigen_LIBRARIES} 
                      ${catkin_LIBRARIES}
                     )

add_executable(ymapping src/ymapping_node.cpp)
add_dependencies(ymapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
                      ymapping
                      sensor
                      map 
                      transform
                      ploc
                      ${Eigen_LIBRARIES} 
                      ${catkin_LIBRARIES}
                     )
                     
add_executable(bresenham src/bresenham.cpp)    
add_dependencies(bresenham ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bresenham ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS} )          
