cmake_minimum_required(VERSION 2.8.3)
project(bmcl)


find_package(catkin REQUIRED COMPONENTS
  message_filters
  nav_msgs
  roscpp
  std_srvs
  tf
)
set(CMAKE_CXX_FLAGS "-std=c++11")
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
#find_package(Ceres REQUIRED)
#include_directories($(CERES_INCLUDES_DIRS))


catkin_package(
    CATKIN_DEPENDS
        rosbag
        roscpp
        tf
  CATKIN_DEPENDS nav_msgs std_srvs
  INCLUDE_DIRS include
  LIBRARIES bmcl_pf
)
FIND_PACKAGE( OpenCV REQUIRED )
## very crucial to add "include_directories(include)"
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} 
		    ${Boost_INCLUDE_DIRS}
		    ${EIGEN_INCLUDE_DIRS}
		#    ${CERES_INCLUDE_DIRS}
		   )

add_library(bmcl_pf  src/bmcl/pf/pf.cpp
                     src/bmcl/pf/pf_pdf.c
                     src/bmcl/pf/pf_vector.c
                     src/bmcl/pf/eig3.c
	   )
#target_link_libraries(bmcl_pf ${CERES_LIBRARIES})	   
add_library(bmcl_map src/bmcl/map/map.c
                     src/bmcl/map/map_cspace.cpp
                     src/bmcl/map/map_range.c
	   )

#add_library(bmcl_sensors
#                    src/bmcl/sensors/bmcl_sensor.cpp
#                    src/bmcl/sensors/bmcl_odom.cpp
#                    src/bmcl/sensors/bmcl_laser.cpp)
#target_link_libraries(bmcl_sensors bmcl_map bmcl_pf)

## Declare a C++ executable
 add_executable(bmcl src/bmcl_node.cpp)
 add_dependencies(bmcl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(
 		       bmcl 
                       bmcl_map 
                       bmcl_pf 
                       ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS} 
                       #${CERES_LIBRARIES}
                       )
                       
 add_executable(bmcl_test src/bmcl_test.cpp)
 add_dependencies(bmcl_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(bmcl_test ${Boost_LIBRARIES} ${catkin_LIBRARIES})

 add_executable(data_test src/data_test.cpp)
 add_dependencies(data_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(data_test ${Boost_LIBRARIES} ${catkin_LIBRARIES})

